#define DT_DRV_COMPAT broadchip_bct3640

#include <zephyr/drivers/display.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

LOG_MODULE_REGISTER(bct3640, CONFIG_DISPLAY_LOG_LEVEL);

/* BCT3640 display dimensions */
#define BCT3640_SEGMENTS_PER_GRID 8
#define BCT3640_GRIDS_PER_DEVICE  16

/* BCT3640 registers and fields */
#define BCT3640_REG_DATA_SET      0x01  /* Data setting register */
#define BCT3640_REG_DISPLAY_CTL   0x02  /* Display control register */
#define BCT3640_REG_PWM           0x03  /* PWM control register */

#define BCT3640_DISPLAY_CTL_LED_OFF   0x00
#define BCT3640_DISPLAY_CTL_LED_ON    0x01

/* BCT3640 command and data headers */
#define BCT3640_CMD_HEADER        0x80
#define BCT3640_DATA_HEADER       0xA0

/**
 * @brief BCT3640 configuration structure
 *
 * This structure contains the device configuration.
 */
struct bct3640_config {
    struct spi_dt_spec spi;     /* SPI bus specification */
    uint32_t num_cascading;     /* Number of cascaded BCT3640 devices */
    uint8_t intensity;          /* Initial brightness intensity */
};

/**
 * @brief BCT3640 data structure
 *
 * This structure contains the device data.
 */
struct bct3640_data {
    uint8_t *grid_buf;          /* Buffer to store grid data */
    uint8_t *tx_buf;            /* Buffer for SPI transmission */
};

/**
 * @brief Transmit a command to all cascaded BCT3640 devices
 *
 * @param dev Pointer to the device structure
 * @param addr Register address
 * @param value Value to write to the register
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_transmit_all(const struct device *dev, const uint8_t addr, const uint8_t value)
{
    const struct bct3640_config *dev_config = dev->config;
    struct bct3640_data *dev_data = dev->data;

    const struct spi_buf tx_buf = {
        .buf = dev_data->tx_buf,
        .len = dev_config->num_cascading * 2,
    };
    const struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1U,
    };

    /* Prepare the command for all cascaded devices */
    for (int i = 0; i < dev_config->num_cascading; i++) {
        dev_data->tx_buf[i * 2] = BCT3640_CMD_HEADER | addr;
        dev_data->tx_buf[i * 2 + 1] = value;
    }

    return spi_write_dt(&dev_config->spi, &tx_bufs);
}

/**
 * @brief Transmit a command to a specific BCT3640 device in the cascade
 *
 * @param dev Pointer to the device structure
 * @param bct3640_idx Index of the target BCT3640 device in the cascade
 * @param addr Register address
 * @param value Value to write to the register
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_transmit_one(const struct device *dev, const uint8_t bct3640_idx,
                const uint8_t addr, const uint8_t value)
{
    const struct bct3640_config *dev_config = dev->config;
    struct bct3640_data *dev_data = dev->data;

    const struct spi_buf tx_buf = {
        .buf = dev_data->tx_buf,
        .len = dev_config->num_cascading * 2,
    };
    const struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1U,
    };

    /* Prepare the command for the specific device, NOP for others */
    for (int i = 0; i < dev_config->num_cascading; i++) {
        if (i != (dev_config->num_cascading - 1 - bct3640_idx)) {
            dev_data->tx_buf[i * 2] = BCT3640_CMD_HEADER;
            dev_data->tx_buf[i * 2 + 1] = 0x00;
            continue;
        }

        dev_data->tx_buf[i * 2] = BCT3640_CMD_HEADER | addr;
        dev_data->tx_buf[i * 2 + 1] = value;
    }

    return spi_write_dt(&dev_config->spi, &tx_bufs);
}

/**
 * @brief Write data to the BCT3640 display
 *
 * This function writes pixel data to the display, updating the specified area.
 *
 * @param dev Pointer to the device structure
 * @param x X coordinate of the display area to update
 * @param y Y coordinate of the display area to update
 * @param desc Pointer to the display buffer descriptor
 * @param buf Pointer to the pixel data buffer
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_write(const struct device *dev, const uint16_t x, const uint16_t y,
             const struct display_buffer_descriptor *desc, const void *buf)
{
    const struct bct3640_config *dev_config = dev->config;
    struct bct3640_data *dev_data = dev->data;

    const uint16_t max_width = BCT3640_SEGMENTS_PER_GRID;
    const uint16_t max_height = dev_config->num_cascading * BCT3640_GRIDS_PER_DEVICE;

    /* Validate input parameters */
    __ASSERT((desc->pitch * desc->height) <= (desc->buf_size * 8U), "Input buffer too small");
    __ASSERT(desc->width <= desc->pitch, "Pitch is smaller than width");
    __ASSERT(desc->pitch <= max_width, "Pitch in descriptor is larger than screen size");
    __ASSERT(desc->height <= max_height, "Height in descriptor is larger than screen size");
    __ASSERT(x + desc->pitch <= max_width,
         "Writing outside screen boundaries in horizontal direction");
    __ASSERT(y + desc->height <= max_height,
         "Writing outside screen boundaries in vertical direction");

    if (desc->width > desc->pitch || (desc->pitch * desc->height) > (desc->buf_size * 8U)) {
        return -EINVAL;
    }

    if ((x + desc->pitch) > max_width || (y + desc->height) > max_height) {
        return -EINVAL;
    }

    const uint16_t end_x = x + desc->width;
    const uint16_t end_y = y + desc->height;
    const uint8_t *byte_buf = buf;

    /* Update the display grid by grid */
    for (uint16_t py = y; py < end_y; ++py) {
        const uint8_t bct3640_idx = py / BCT3640_GRIDS_PER_DEVICE;
        const uint8_t grid_idx = py % BCT3640_GRIDS_PER_DEVICE;
        uint8_t segment = 0;

        /* Construct the segment data for the current grid */
        for (uint16_t px = x; px < end_x; ++px) {
            if (byte_buf[py * desc->pitch + px / 8] & BIT(7 - (px % 8))) {
                segment |= BIT(px);
            }
        }

        /* Send the segment data to the appropriate BCT3640 device */
        int ret = bct3640_transmit_one(dev, bct3640_idx, BCT3640_REG_DATA_SET + grid_idx, segment);
        if (ret < 0) {
            return ret;
        }

        /* Store the updated segment data */
        dev_data->grid_buf[py] = segment;
    }

    return 0;
}

/**
 * @brief Set the brightness of the BCT3640 display
 *
 * @param dev Pointer to the device structure
 * @param brightness Brightness value (0-255)
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_set_brightness(const struct device *dev, const uint8_t brightness)
{
    /* BCT3640 supports 16 levels of brightness (0-15), so we scale down the input */
    return bct3640_transmit_all(dev, BCT3640_REG_PWM, brightness >> 4);
}

/**
 * @brief Set the pixel format of the BCT3640 display
 *
 * @param dev Pointer to the device structure
 * @param format Pixel format to set
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_set_pixel_format(const struct device *dev,
                    const enum display_pixel_format format)
{
    ARG_UNUSED(dev);

    /* BCT3640 only supports MONO01 format */
    switch (format) {
    case PIXEL_FORMAT_MONO01:
        return 0;
    default:
        return -ENOTSUP;
    }
}

/**
 * @brief Set the orientation of the BCT3640 display
 *
 * @param dev Pointer to the device structure
 * @param orientation Orientation to set
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_set_orientation(const struct device *dev,
                   const enum display_orientation orientation)
{
    ARG_UNUSED(dev);

    /* BCT3640 only supports normal orientation */
    switch (orientation) {
    case DISPLAY_ORIENTATION_NORMAL:
        return 0;
    default:
        return -ENOTSUP;
    }
}

/**
 * @brief Get the capabilities of the BCT3640 display
 *
 * @param dev Pointer to the device structure
 * @param caps Pointer to the capabilities structure to fill
 */
static void bct3640_get_capabilities(const struct device *dev, struct display_capabilities *caps)
{
    const struct bct3640_config *dev_config = dev->config;

    caps->x_resolution = BCT3640_SEGMENTS_PER_GRID;
    caps->y_resolution = BCT3640_GRIDS_PER_DEVICE * dev_config->num_cascading;
    caps->supported_pixel_formats = PIXEL_FORMAT_MONO01;
    caps->screen_info = 0;
    caps->current_pixel_format = PIXEL_FORMAT_MONO01;
    caps->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

/* BCT3640 display driver API structure */
static const struct display_driver_api bct3640_api = {
    .write = bct3640_write,
    .set_brightness = bct3640_set_brightness,
    .get_capabilities = bct3640_get_capabilities,
    .set_pixel_format = bct3640_set_pixel_format,
    .set_orientation = bct3640_set_orientation,
};

/**
 * @brief Initialize the BCT3640 display
 *
 * This function initializes the BCT3640 display, setting up the SPI communication
 * and configuring the initial display state.
 *
 * @param dev Pointer to the device structure
 * @return 0 if successful, negative errno code if failure
 */
static int bct3640_init(const struct device *dev)
{
    const struct bct3640_config *dev_config = dev->config;
    struct bct3640_data *dev_data = dev->data;
    int ret;

    if (!spi_is_ready_dt(&dev_config->spi)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    /* Turn off all LEDs initially */
    memset(dev_data->grid_buf, 0,
           dev_config->num_cascading * BCT3640_GRIDS_PER_DEVICE * sizeof(uint8_t));

    /* Turn on the display */
    ret = bct3640_transmit_all(dev, BCT3640_REG_DISPLAY_CTL, BCT3640_DISPLAY_CTL_LED_ON);
    if (ret < 0) {
        LOG_ERR("Failed to turn on display");
        return ret;
    }

    /* Set initial brightness */
    ret = bct3640_transmit_all(dev, BCT3640_REG_PWM, dev_config->intensity);
    if (ret < 0) {
        LOG_ERR("Failed to set global brightness");
        return ret;
    }

    /* Clear the display */
    const struct display_buffer_descriptor desc = {
        .buf_size = dev_config->num_cascading * BCT3640_GRIDS_PER_DEVICE,
        .height = dev_config->num_cascading * BCT3640_GRIDS_PER_DEVICE,
        .width = BCT3640_SEGMENTS_PER_GRID,
        .pitch = BCT3640_SEGMENTS_PER_GRID,
    };

    ret = bct3640_write(dev, 0, 0, &desc, dev_data->grid_buf);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/* BCT3640 device initialization macro */
#define DISPLAY_BCT3640_INIT(n)                                                \
    static uint8_t bct3640_grid_data_##n[DT_INST_PROP(n, num_cascading) *      \
                          BCT3640_GRIDS_PER_DEVICE];                           \
    static uint8_t bct3640_tx_buf##n[DT_INST_PROP(n, num_cascading) * 2];      \
    static struct bct3640_data bct3640_data_##n = {                            \
        .grid_buf = bct3640_grid_data_##n,                                     \
        .tx_buf = bct3640_tx_buf##n,                                           \
    };                                                                         \
    static const struct bct3640_config bct3640_config_##n = {                  \
        .spi = SPI_DT_SPEC_INST_GET(                                           \
            n, SPI_OP_MODE_MASTER | SPI_WORD_SET(8U), 0U),                     \
        .num_cascading = DT_INST_PROP(n, num_cascading),                       \
        .intensity = DT_INST_PROP(n, intensity),                               \
    };                                                                         \
    DEVICE_DT_INST_DEFINE(n, bct3640_init, NULL, &bct3640_data_##n,            \
              &bct3640_config_##n, POST_KERNEL,                                \
              CONFIG_DISPLAY_INIT_PRIORITY, &bct3640_api);

DT_INST_FOREACH_STATUS_OKAY(DISPLAY_BCT3640_INIT)