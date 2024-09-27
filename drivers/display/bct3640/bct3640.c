#define DT_DRV_COMPAT broadchip_bct3640

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bct3640, CONFIG_DISPLAY_LOG_LEVEL);

/* BCT3640 display dimensions */
#define BCT3640_SEGMENTS_PER_GRID 8
#define BCT3640_GRIDS_PER_DEVICE 16

/* BCT3640 command types */
#define BCT3640_CMD_MODE_SETTING 0x40    // 01xxxxxx
#define BCT3640_CMD_DISPLAY_CONTROL 0x80 // 10xxxxxx
#define BCT3640_CMD_ADDRESS_SETTING 0xC0 // 11xxxxxx

/* Mode Setting Command fields */
#define BCT3640_MODE_ADDR_INCREMENT (0 << 2)
#define BCT3640_MODE_ADDR_FIXED (1 << 2)
#define BCT3640_MODE_NORMAL (0 << 3)
#define BCT3640_MODE_TEST (1 << 3)

/* Display ON/OFF */
#define BCT3640_DISPLAY_OFF 0x00
#define BCT3640_DISPLAY_ON 0x08

/* Pulse Width Modulation (PWM) settings */
#define BCT3640_DISPLAY_PWM_1_16 0x00
#define BCT3640_DISPLAY_PWM_2_16 0x01
#define BCT3640_DISPLAY_PWM_4_16 0x02
#define BCT3640_DISPLAY_PWM_10_16 0x03
#define BCT3640_DISPLAY_PWM_11_16 0x04
#define BCT3640_DISPLAY_PWM_12_16 0x05
#define BCT3640_DISPLAY_PWM_13_16 0x06
#define BCT3640_DISPLAY_PWM_14_16 0x07

/* Mask for PWM bits */
#define BCT3640_DISPLAY_PWM_MASK 0x07

/**
 * @brief BCT3640 configuration structure
 */
struct bct3640_config
{
  struct spi_dt_spec spi;
};

struct bct3640_data
{
  uint8_t buffer[BCT3640_GRIDS_PER_DEVICE]; // Display buffer
};

static int bct3640_write_command(const struct device *dev, uint8_t cmd)
{
  const struct bct3640_config *config = dev->config;
  struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
  struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

  return spi_write_dt(&config->spi, &tx_buf_set);
}

static int bct3640_write_data(const struct device *dev, uint8_t addr,
                              const uint8_t *data, size_t len)
{
  const struct bct3640_config *config = dev->config;
  struct bct3640_data *driver_data = dev->data;

  if (addr >= BCT3640_GRIDS_PER_DEVICE) // addr should be less than BCT3640_GRIDS_PER_DEVICE
  {
    return -EINVAL;
  }
  if (addr + len > BCT3640_GRIDS_PER_DEVICE) // addr + len should be less than BCT3640_GRIDS_PER_DEVICE
  {
    LOG_WRN("%d (addr + len) should be less than %d", addr + len, BCT3640_GRIDS_PER_DEVICE);
    LOG_WRN("Data will be truncated");
    len = BCT3640_GRIDS_PER_DEVICE - addr;
  }

  int ret = bct3640_write_command(dev, BCT3640_CMD_ADDRESS_SETTING | addr);
  if (ret < 0)
  {
    return ret;
  }

  struct spi_buf tx_buf = {.buf = (void *)data, .len = len};
  struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};

  ret = spi_write_dt(&config->spi, &tx_buf_set);
  if (ret < 0)
  {
    return ret;
  }

  memcpy(&driver_data->buffer[addr], data, len);
  return 0;
}

static int bct3640_write(const struct device *dev, 
                         const uint16_t x, const uint16_t y,
                         const struct display_buffer_descriptor *desc,
                         const void *buf)
{
  if (x != 0 || y >= BCT3640_GRIDS_PER_DEVICE ||
      desc->width != BCT3640_SEGMENTS_PER_GRID ||
      y + desc->height > BCT3640_GRIDS_PER_DEVICE)
  {
    return -EINVAL;
  }

  return bct3640_write_data(dev, y, buf, desc->height);
}

static int bct3640_read(const struct device *dev, const uint16_t x,
                        const uint16_t y,
                        const struct display_buffer_descriptor *desc,
                        void *buf)
{
  const struct bct3640_data *data = dev->data;

  if (x != 0 || y >= BCT3640_GRIDS_PER_DEVICE ||
      desc->width != BCT3640_SEGMENTS_PER_GRID ||
      y + desc->height > BCT3640_GRIDS_PER_DEVICE)
  {
    return -EINVAL;
  }

  memcpy(buf, &data->buffer[y], desc->height);
  return 0;
}

static void *bct3640_get_framebuffer(const struct device *dev)
{ 
  return NULL; 
}

static int bct3640_set_brightness(const struct device *dev, const uint8_t brightness)
{
  uint8_t pwm_setting = brightness & BCT3640_DISPLAY_PWM_MASK;
  return bct3640_write_command(dev, BCT3640_CMD_DISPLAY_CONTROL |
                                        BCT3640_DISPLAY_ON | pwm_setting);
}

static int bct3640_set_contrast(const struct device *dev,
                                const uint8_t contrast)
{
  return -ENOTSUP;
}

static void bct3640_get_capabilities(const struct device *dev,
                         struct display_capabilities *capabilities)
{
  memset(capabilities, 0, sizeof(struct display_capabilities));
  capabilities->x_resolution = BCT3640_SEGMENTS_PER_GRID;
  capabilities->y_resolution = BCT3640_GRIDS_PER_DEVICE;
  capabilities->supported_pixel_formats = PIXEL_FORMAT_MONO01;
  capabilities->current_pixel_format = PIXEL_FORMAT_MONO01;
  capabilities->screen_info = SCREEN_INFO_MONO_VTILED;
}

static int bct3640_set_pixel_format(const struct device *dev, const enum display_pixel_format pixel_format)
{
  if (pixel_format == PIXEL_FORMAT_MONO01)
  {
    return 0;
  }
  return -ENOTSUP;
}

static int bct3640_init(const struct device *dev)
{
  const struct bct3640_config *config = dev->config;
  struct bct3640_data *data = dev->data;

  // Check if SPI is ready
  if (!spi_is_ready_dt(&config->spi))
  {
    LOG_ERR("SPI is not ready");
    return -ENODEV;
  }

  // Initialize BCT3640
  // Set mode to normal and address increment
  int ret = bct3640_write_command(dev,  BCT3640_CMD_MODE_SETTING |
                                        BCT3640_MODE_ADDR_INCREMENT |
                                        BCT3640_MODE_NORMAL);
  if (ret < 0)
  {
    LOG_ERR("Failed to set mode");
    return ret;
  }
  // Turn on display with PWM 14/16
  ret = bct3640_write_command(dev,  BCT3640_CMD_DISPLAY_CONTROL |
                                    BCT3640_DISPLAY_ON |
                                    BCT3640_DISPLAY_PWM_14_16);
  if (ret < 0)
  {
    LOG_ERR("Failed to turn on display");
    return ret;
  }

  // Clear display
  memset(data->buffer, 0, sizeof(data->buffer));
  ret = bct3640_write_data(dev, 0, data->buffer, BCT3640_GRIDS_PER_DEVICE);
  if (ret < 0)
  {
    LOG_ERR("Failed to clear display");
    return ret;
  }

  LOG_INF("BCT3640 initialized");
  return 0;
}

static const struct display_driver_api bct3640_api = {
    .write = bct3640_write,
    .read = bct3640_read,
    .get_framebuffer = bct3640_get_framebuffer,
    .set_brightness = bct3640_set_brightness,
    .set_contrast = bct3640_set_contrast,
    .get_capabilities = bct3640_get_capabilities,
    .set_pixel_format = bct3640_set_pixel_format,
};

#define BCT3640_INIT(inst)                                                     \
  static const struct bct3640_config bct3640_config_##inst = {                 \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0),\
  };                                                                           \
                                                                               \
  static struct bct3640_data bct3640_data_##inst;                              \
                                                                               \
  DEVICE_DT_INST_DEFINE(inst, bct3640_init, NULL, &bct3640_data_##inst,        \
                        &bct3640_config_##inst, POST_KERNEL,                   \
                        CONFIG_DISPLAY_INIT_PRIORITY, &bct3640_api);

DT_INST_FOREACH_STATUS_OKAY(BCT3640_INIT)