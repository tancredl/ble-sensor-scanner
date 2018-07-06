// BLE Sensor scanner, based on code from
// Intel Edison Playground / Copyright (c) 2015 Damian Kołakowski. All rights reserved.

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <curses.h>
#include <errno.h>
#include <gflags/gflags.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

DEFINE_double(report_interval_s, 30.0,
              "Per-device interval between measurement reports.");

namespace {

// Executes an HCI request successfully or exits.
void ExecHciRequestOrDie(uint16_t ocf, int clen, void *cparam, const int device,
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
  }
}

} // namespace

short read_short(unsigned char *buf) { return *buf + (*(buf + 1) << 8); }

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, /*remove_flags=*/false);
  int ret, status;

  // Get HCI device.

  const int device = hci_open_dev(hci_get_route(NULL));
  if (device < 0) {
    perror("Failed to open HCI device.");
    return 0;
  }

  // Set BLE scan parameters.
  le_set_scan_parameters_cp scan_params_cp{};
  scan_params_cp.type = 0x00;
  scan_params_cp.interval = htobs(0x0010);
  scan_params_cp.window = htobs(0x0010);
  scan_params_cp.own_bdaddr_type = 0x00; // Public Device Address (default).
  scan_params_cp.filter = 0x00;          // Accept all.
  ExecHciRequestOrDie(OCF_LE_SET_SCAN_PARAMETERS,
                      LE_SET_SCAN_PARAMETERS_CP_SIZE, &scan_params_cp, device,
                      "Failed to set scan parameters data.");

  // Set BLE events report mask.
  le_set_event_mask_cp event_mask_cp{};
  memset(event_mask_cp.mask, 0xff, 8); // All events
  ExecHciRequestOrDie(OCF_LE_SET_EVENT_MASK, LE_SET_EVENT_MASK_CP_SIZE,
                      &event_mask_cp, device, "Failed to set event mask.");

  // Enable scanning.
  le_set_scan_enable_cp scan_cp{};
  scan_cp.enable = 0x01;     // Enable flag.
  scan_cp.filter_dup = 0x00; // Filtering disabled.
  ExecHciRequestOrDie(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE,
                      &scan_cp, device, "Failed to enable scan.");

  // Get Results.
  struct hci_filter nf;
  hci_filter_clear(&nf);
  hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
  hci_filter_set_event(EVT_LE_META_EVENT, &nf);
  if (setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
    hci_close_dev(device);
    perror("Could not set socket options\n");
    return 0;
  }

  printf("Scanning (report interval=%.1fs)...\n", FLAGS_report_interval_s);

  uint8_t buf[HCI_MAX_EVENT_SIZE];
  evt_le_meta_event *meta_event;
  le_advertising_info *info;
  int len;

  while (1) {
    len = read(device, buf, sizeof(buf));
    if (len >= HCI_EVENT_HDR_SIZE) {
      meta_event = (evt_le_meta_event *)(buf + HCI_EVENT_HDR_SIZE + 1);
      if (meta_event->subevent == EVT_LE_ADVERTISING_REPORT) {
        uint8_t reports_count = meta_event->data[0];
        void *offset = meta_event->data + 1;
        while (reports_count--) {
          info = (le_advertising_info *)offset;
          char addr[18];
          ba2str(&(info->bdaddr), addr);
          printf("%s - RSSI %4d ", addr, (char)info->data[info->length]);
          int i;
          for (i = 0; i < info->length + 2; ++i) {
            printf("%02X", *(info->data + i));
          }
          float voltage = 0.0;
          if (info->length > 15) {
            voltage = read_short(info->data + 15) / 1000.0;
          }
          printf(" T=%3.2f C H=%3.2f %% Vdd=%1.3f V",
                 ((float)read_short(info->data + 11)) / 256.0,
                 ((float)read_short(info->data + 13)) / 256.0, voltage);
          printf("\n");
          fflush(stdout);
          offset = info->data + info->length + 2;
        }
      }
    }
  }

  // Disable scanning.
  le_set_scan_enable_cp end_scan_cp{};
  end_scan_cp.enable = 0x00; // Disable flag.
  ExecHciRequestOrDie(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE,
                      &end_scan_cp, device, "Failed to disable scan.");

  hci_close_dev(device);

  return 0;
}
