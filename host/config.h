// struct for config

struct pmlps_config
{
  // address and port
  int lps_port;
  const char *tlm_addr;
  int tlm_port;
  // cam
  int cam_image_width;
  int cam_image_height;
  float cam_lens_ratio;
  bool cam_lens_fisheye;
  float cam_height;
  float cam_direction;
  // marker
  int marker_type;
  float marker_sqsize;
};

extern struct pmlps_config config;

// Default values.

// PMLPS server udp port
#define LPS_PORT 5770

// APM telemetry tcp
#define TLM_ADDR "192.168.11.1"
#define TLM_PORT 5900

// CAM x-axis from north
#define CAM_DIRECTION 2.6

// CAM height in cm
#define CAM_HEIGHT 220.0

// CAM image resolution type
#define CAM_IMAGE_WIDTH 320
#define CAM_IMAGE_HEIGHT 240

// Lens ratio
#define CAM_LENS_RATIO 0.003

// Marker type
#define MARKER_TYPE_I 1

// Square size of marker surround
#define SQ_SIZEOF_SURROUND 104.0
