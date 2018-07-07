// BLE Sensor scanner, based on code from
// Intel Edison Playground / Copyright (c) 2015 Damian Kołakowski. All rights
// reserved.

#include <map>
#include <string>
#include <vector>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <curses.h>
#include <errno.h>
#include <gflags/gflags.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

DEFINE_double(report_interval_s, 30.0,
              "Per-device interval between measurement reports.");
DEFINE_int32(
    device_id, -1,
    "Bluetooth adapter id (0 being the first). Use -1 for first available.");

using std::string;
using Bytes = std::vector<uint8_t>;

// Needed for map of bt addresses.
bool operator<(const bdaddr_t a, const bdaddr_t b) {
  return memcmp(&a, &b, sizeof(bdaddr_t)) < 0;
}

namespace {

// Readings from a sensor.
class SensorReadings {
public:
  void AddReading(const le_advertising_info &info, const uint8_t *data) {
    readings.push_back(std::pair<le_advertising_info, Bytes>(
        info, Bytes(data, data + info.length + 1)));
  }

  void PrintAndFlush() {
    for (const std::pair<le_advertising_info, Bytes> &info_and_data :
         readings) {
      const le_advertising_info &info = info_and_data.first;
      const Bytes &data = info_and_data.second;

      char addr[18]{};
      ba2str(&(info.bdaddr), addr);
      printf("%s - RSSI %4d ", addr, (char)data[info.length]);
      for (const auto &byte : data) {
        printf("%02X", byte);
      }
      float voltage = 0.0;
      if (info.length > 15) {
        voltage = read_short(data, 15) / 1000.0;
      }
      printf(" T=%3.2f C H=%3.2f %% Vdd=%1.3f V",
             ((float)read_short(data, 11)) / 256.0,
             ((float)read_short(data, 13)) / 256.0, voltage);
      printf("\n");
    }
    readings.clear();
    fflush(stdout);
  }

private:
  short read_short(const Bytes &data, int offset) {
    return data[offset] + (data[offset + 1] << 8);
  }

  // Readings as pairs of advertising info and user bytes.
  std::vector<std::pair<le_advertising_info, Bytes>> readings;
  // TODO(tancred): Implement this.
  double mute_until = 0.0;
};

// Class implementing BLE sensor scanning by receiving and decoding
// advertisement packets.
class BLESensorScanner {

public:
  BLESensorScanner() = delete;

  ~BLESensorScanner() {
    // Disable scanning.
    le_set_scan_enable_cp end_scan_cp{};
    end_scan_cp.enable = 0x00; // Disable flag.
    ExecHciRequestOrDie(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE,
                        &end_scan_cp, "Failed to disable scan.");

    hci_close_dev(device);
  }

  static BLESensorScanner CreateFromDeviceIdOrDie(const int device_id) {
    const int device =
        hci_open_dev(device_id < 0 ? hci_get_route(NULL) : device_id);
    if (device < 0) {
      perror("Failed to open HCI device.");
      exit(-1);
    }
    return BLESensorScanner(device);
  }

  // Initialize scanning. Need to be done once before reding sensor data.
  void InitScanningOrDie() {
    // Set BLE scan parameters.
    le_set_scan_parameters_cp scan_params_cp{};
    scan_params_cp.type = 0x00;
    scan_params_cp.interval = htobs(0x0010);
    scan_params_cp.window = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00; // Public Device Address (default).
    scan_params_cp.filter = 0x00;          // Accept all.
    ExecHciRequestOrDie(OCF_LE_SET_SCAN_PARAMETERS,
                        LE_SET_SCAN_PARAMETERS_CP_SIZE, &scan_params_cp,
                        "Failed to set scan parameters data.");

    // Set BLE events report mask.
    le_set_event_mask_cp event_mask_cp{};
    memset(event_mask_cp.mask, 0xff, 8); // All events
    ExecHciRequestOrDie(OCF_LE_SET_EVENT_MASK, LE_SET_EVENT_MASK_CP_SIZE,
                        &event_mask_cp, "Failed to set event mask.");

    // Enable scanning.
    le_set_scan_enable_cp scan_cp{};
    scan_cp.enable = 0x01;     // Enable flag.
    scan_cp.filter_dup = 0x00; // Filtering disabled.
    ExecHciRequestOrDie(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE,
                        &scan_cp, "Failed to enable scan.");

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
      hci_close_dev(device);
      perror("Could not set socket options\n");
      exit(-1);
    }
  }

  // Receive a sensor reading. Blocks until there is a reading.
  void ReceiveReading() {
    uint8_t buf[HCI_MAX_EVENT_SIZE];
    int len = read(device, buf, sizeof(buf));
    if (len >= HCI_EVENT_HDR_SIZE) {
      evt_le_meta_event *meta_event =
          (evt_le_meta_event *)(buf + HCI_EVENT_HDR_SIZE + 1);
      if (meta_event->subevent == EVT_LE_ADVERTISING_REPORT) {
        uint8_t reports_count = meta_event->data[0];
        uint8_t *offset = meta_event->data + 1;
        while (reports_count--) {
          le_advertising_info &info = *((le_advertising_info *)offset);
          readings[info.bdaddr].AddReading(info, info.data);
          offset += EVT_LE_META_EVENT_SIZE + LE_ADVERTISING_INFO_SIZE +
                    info.length +
                    1 /* final RSSI byte in EVT_LE_ADVERTISING_REPORT */;
        }
      }
    }
  }

  // Prints all received readings and flushes the printed readings.
  void PrintAndFlushReadings() {
    for (std::pair<const bdaddr_t, SensorReadings> &keyed_reading : readings) {
      keyed_reading.second.PrintAndFlush();
    }
  }

private:
  BLESensorScanner(const int device) : device(device){};

  // Executes an HCI request successfully or exits.
  void ExecHciRequestOrDie(uint16_t ocf, int clen, void *cparam,
                           const char *error) {
    int status;
    hci_request rq{};
    rq.ogf = OGF_LE_CTL;
    rq.ocf = ocf;
    rq.cparam = cparam;
    rq.clen = clen;
    rq.rparam = &status;
    rq.rlen = 1;
    if (hci_send_req(device, &rq, 1000) < 0) {
      hci_close_dev(device);
      perror(error);
      exit(-1);
    }
  }

  const int device; // The device id.
  std::map<bdaddr_t, SensorReadings>
      readings; // Readings keyed by sender BT address.
};

} // namespace

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/false);

  static bool interrupted = false;
  signal(SIGINT, [](int unused) { interrupted = true; });

  BLESensorScanner scanner = BLESensorScanner::CreateFromDeviceIdOrDie(
      FLAGS_device_id < 0 ? hci_get_route(NULL) : FLAGS_device_id);
  scanner.InitScanningOrDie();

  printf("Scanning (report interval=%.1fs)...\n", FLAGS_report_interval_s);
  while (!interrupted) {
    scanner.ReceiveReading();
    scanner.PrintAndFlushReadings();
  }

  printf("Scan ended.\n");
  return 0;
}
