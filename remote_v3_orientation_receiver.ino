#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// This device is the ESP-NOW receiver at MAC 64:E8:33:86:D6:DC.
// There's no wireless network in this environment, so WiFi is only used to
// bring up the radio for ESP-NOW -- no AP is ever joined. Received
// orientation packets are framed and written out over the native USB CDC
// serial port to a connected host.
//
// Wire framing on the USB CDC link:
//   [payload bytes][CRC-8 over payload] -> COBS-encode -> [encoded][0x00]
// The trailing 0x00 is the frame delimiter; COBS guarantees the encoded
// bytes are all non-zero so the host can resynchronize on any 0x00.

// Both this device and the sender must be pinned to the same channel, since
// neither one joins an AP that would otherwise fix the channel for them.
const uint8_t WIFI_CHANNEL = 1;

// The receiver's own device id, stamped into the heartbeat frames so the host
// can tell them apart from forwarded sender packets.
const uint8_t RECEIVER_DEVICE_ID = 0;

// Heartbeat cadence and the deviceType value that marks a frame as a
// receiver heartbeat rather than a forwarded orientation packet.
const uint32_t HEARTBEAT_INTERVAL_MS = 500;
const uint8_t HEARTBEAT_DEVICE_TYPE = 5;

// Set to 1 to print human-readable debug lines to Serial (e.g. to confirm a
// sender is transmitting, using the Arduino Serial Monitor). This shares the
// same USB CDC stream as the binary framed protocol, so leave it at 0
// whenever a host is actually parsing that protocol. When it is 1 the
// forwarding and heartbeat tasks are not started, so nothing writes binary.
#define DEBUG_SERIAL 0

typedef struct __attribute__((packed)) {
  uint8_t id;
  uint32_t time;
  uint8_t deviceType;
  uint8_t seq;
  int16_t w;
  int16_t x;
  int16_t y;
  int16_t z;
  uint8_t action_flag;
} orientation_packet_t;

// The heartbeat payload is a strict prefix of the orientation packet: just
// the id, timestamp, deviceType, and sequence number.
typedef struct __attribute__((packed)) {
  uint8_t id;
  uint32_t time;
  uint8_t deviceType;
  uint8_t seq;
} heartbeat_packet_t;

// The largest raw payload that flows through the queue (an orientation packet).
static const size_t MAX_FRAME_PAYLOAD = sizeof(orientation_packet_t);

// Room for the payload plus the CRC byte, COBS worst-case overhead (one code
// byte per 254 data bytes plus the leading code byte), and the 0x00 delimiter.
static const size_t MAX_ENCODED = MAX_FRAME_PAYLOAD + 1 + 2 + 1;

// A payload copied off the radio callback / heartbeat timer, waiting to be
// framed and written to USB by the forwarding task.
typedef struct {
  uint8_t len;
  uint8_t data[MAX_FRAME_PAYLOAD];
} queued_frame_t;

static const size_t FRAME_QUEUE_LENGTH = 16;
static QueueHandle_t frame_queue = NULL;

// CRC-8, polynomial 0x07 (CRC-8/SMBUS), init 0x00, no reflection or final xor.
static uint8_t crc8(const uint8_t *data, size_t len)
{
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// Standard Consistent Overhead Byte Stuffing. Encodes `length` input bytes
// into `output` (which the caller sizes for the worst case) and returns the
// number of encoded bytes, not including any frame delimiter.
static size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output)
{
  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;

  while (read_index < length) {
    if (input[read_index] == 0) {
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    } else {
      output[write_index++] = input[read_index++];
      code++;
      if (code == 0xFF) {
        output[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
  }
  output[code_index] = code;
  return write_index;
}

// Frame a payload (append CRC-8, COBS-encode, add the 0x00 delimiter) and
// write it to USB CDC. The frame is dropped -- never blocked -- when the host
// is not connected or the CDC transmit buffer cannot take the whole frame
// right now, so a stalled or absent host can never back up this task.
static void forward_payload(const uint8_t *payload, size_t len)
{
  uint8_t framed[MAX_FRAME_PAYLOAD + 1];
  memcpy(framed, payload, len);
  framed[len] = crc8(payload, len);

  uint8_t encoded[MAX_ENCODED];
  size_t enc_len = cobs_encode(framed, len + 1, encoded);
  encoded[enc_len++] = 0x00;  // frame delimiter

  // availableForWrite() reports free space in the CDC ring buffer; requiring
  // room for the whole frame keeps Serial.write() from blocking. `!Serial` is
  // true when no host has the port open (DTR deasserted).
  if (!Serial || Serial.availableForWrite() < (int)enc_len) {
    return;  // host not connected or would block -- drop the frame
  }
  Serial.write(encoded, enc_len);
}

// Sole owner of the USB CDC writes: drains the queue and frames each payload.
static void forwarding_task(void *arg)
{
  queued_frame_t frame;
  for (;;) {
    if (xQueueReceive(frame_queue, &frame, portMAX_DELAY) == pdTRUE) {
      forward_payload(frame.data, frame.len);
    }
  }
}

// Enqueues a heartbeat payload every HEARTBEAT_INTERVAL_MS so the host can see
// the receiver is alive. Routed through the same queue as forwarded packets so
// the forwarding task remains the single serial writer.
static void heartbeat_task(void *arg)
{
  uint8_t seq = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));

    heartbeat_packet_t hb;
    hb.id = RECEIVER_DEVICE_ID;
    hb.time = millis();
    hb.deviceType = HEARTBEAT_DEVICE_TYPE;
    hb.seq = seq++;

    queued_frame_t frame;
    frame.len = sizeof(hb);
    memcpy(frame.data, &hb, sizeof(hb));
    xQueueSend(frame_queue, &frame, 0);  // drop if the queue is full
  }
}

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *incoming_data, int len)
{
  if (len != sizeof(orientation_packet_t)) {
#if DEBUG_SERIAL
    Serial.printf("Dropping ESP-NOW packet with unexpected size %d\n", len);
#endif
    return;
  }

#if DEBUG_SERIAL
  Serial.printf("Received %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n", len,
                 recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                 recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
#else
  // Keep the radio callback fast: copy the payload into the queue and return.
  // The forwarding task does the CRC/COBS work and the USB write. Drop the
  // frame if the queue is full rather than stalling the WiFi task.
  queued_frame_t frame;
  frame.len = (uint8_t)len;
  memcpy(frame.data, incoming_data, len);
  xQueueSend(frame_queue, &frame, 0);
#endif
}

void setup(void)
{
  Serial.begin(115200);

#if ARDUINO_USB_CDC_ON_BOOT
  // On the XIAO ESP32C3 `Serial` is the native USB Serial/JTAG (HWCDC). Make
  // its writes non-blocking: the forwarding task already gates on
  // availableForWrite(), so a full/undrained TX buffer must never stall it.
  Serial.setTxTimeoutMs(0);
#endif

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(on_data_recv);

#if !DEBUG_SERIAL
  frame_queue = xQueueCreate(FRAME_QUEUE_LENGTH, sizeof(queued_frame_t));
  if (frame_queue == NULL) {
    Serial.println("Error creating frame queue");
    return;
  }
  xTaskCreate(forwarding_task, "forwarding", 4096, NULL, 2, NULL);
  xTaskCreate(heartbeat_task, "heartbeat", 2048, NULL, 1, NULL);
#endif

  Serial.println("Setup completed");
}

void loop(void)
{
}
