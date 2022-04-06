
#include <stdint.h>
#include "mavlink_sample_frame.h"
#include <stdlib.h>

enum mavlink_state
{
    Mavlink_Idle,
    Mavlink_read_len,
    Mavlink_read_seq,
    Mavlink_read_sys_id,
    Mavlink_read_comp_id,
    Mavlink_read_msg_id,
    Mavlink_read_payload,
    Mavlink_read_crc_low,
    Mavlink_read_crc_high
};

struct mavlink_config
{
    uint8_t sys_id;
    uint8_t comp_id;
};

struct mavlink_frame
{
    uint8_t len;
    uint8_t seq;
    uint8_t sys_id;
    uint8_t comp_id;
    uint8_t msg_id;
    uint8_t payload;
};

struct mavlink_data
{
    enum mavlink_state state;
    uint16_t crc;
    uint16_t crc_expected;
    uint8_t i_payload;
    uint8_t crc_error;
    uint8_t len;
    struct mavlink_frame *curr_frame;
};

struct device
{
    void *config;
    void *data;
    const char *name;
    struct mavlink_frame *(*read)(const struct device *);
};

void *mavlink_alloc()
{
    return malloc(262);
}

void mavlink_free(void *addr)
{
    free(addr);
}

static uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len);

static inline void crc16(uint16_t *seed, const uint8_t c)
{
    *seed = crc16_ccitt(*seed, &c, 1);
}

struct mavlink_frame *mavlink_parse_byte(const struct device *mavlink_dev, uint8_t c)
{
    struct mavlink_data *data = (struct mavlink_data *)mavlink_dev->data;

    switch (data->state)
    {
    case Mavlink_Idle:
        if (c == 0xFE)
        {
            data->crc = 0;
            data->state = Mavlink_read_len;
            data->crc_error = 0;
            data->i_payload = 0;
            data->curr_frame = mavlink_alloc();
            data->crc_expected = 0;
        }
        break;
    case Mavlink_read_len:
        crc16(&data->crc, c);
        data->len = c;
        data->curr_frame->len = c;
        data->state = Mavlink_read_seq;
        break;
    case Mavlink_read_seq:
        crc16(&data->crc, c);
        data->curr_frame->seq = c;
        data->state = Mavlink_read_sys_id;
        break;
    case Mavlink_read_sys_id:
        crc16(&data->crc, c);
        data->curr_frame->sys_id = c;
        data->state = Mavlink_read_comp_id;
        break;
    case Mavlink_read_comp_id:
        crc16(&data->crc, c);
        data->curr_frame->comp_id = c;
        data->state = Mavlink_read_msg_id;
        break;
    case Mavlink_read_msg_id:
        crc16(&data->crc, c);
        data->curr_frame->msg_id = c;
        data->state = Mavlink_read_payload;
        break;
    case Mavlink_read_payload:
        crc16(&data->crc, c);
        (&data->curr_frame->payload)[data->i_payload] = c;

        if (data->i_payload < data->len - 1)
        {
            data->i_payload++;
        }
        else
        {
            data->i_payload = 0;
            data->state = Mavlink_read_crc_low;
        }

        break;
    case Mavlink_read_crc_low:
        data->crc_expected |= c;
        data->state = Mavlink_read_crc_high;
        break;
    case Mavlink_read_crc_high:
        data->crc_expected |= (c << 8u);
        if (data->crc_expected != data->crc)
        {
            data->crc_error = 1;
            mavlink_free(data->curr_frame);
            data->curr_frame = NULL;
        }
        data->state = Mavlink_Idle;

        return data->curr_frame;
    default:
        break;
    }

    return NULL;
}

static inline struct mavlink_frame *mavlink_read(const struct device *dev)
{
    dev->read(dev);
}

static uint16_t crc16_ccitt(uint16_t seed, const uint8_t *src, size_t len)
{
    for (; len > 0; len--)
    {
        uint8_t e, f;

        e = seed ^ *src++;
        f = e ^ (e << 4);
        seed = (seed >> 8) ^ ((uint16_t)f << 8) ^ ((uint16_t)f << 3) ^ ((uint16_t)f >> 4);
    }

    return seed;
}


//------------------------------------------------------------------
struct mavlink_config config;
struct mavlink_data data;

#include <stdio.h>

struct mavlink_frame *mavlink_test_read(const struct device *dev)
{
    int len = sizeof(sample_frame);
    static int i = 0;

    for (; i < len; i++)
    {
        if (mavlink_parse_byte(dev, sample_frame[i]))
        {
            return (data.curr_frame);
        }
    }

    return NULL;
}

struct device dev = {&config, &data, "mavlink", mavlink_test_read};

void print_frame(struct mavlink_frame *frame)
{
    printf("len: %d, seq: %d, comp_id: %d, msg_id: %d, sys_id: %d\n",
           frame->len, frame->seq, frame->comp_id, frame->msg_id, frame->sys_id);
    for (int i = 0; i < frame->len; i++)
    {
        printf("%4d", (&frame->payload)[i]);
    }
    puts("");
}

int main()
{
    struct mavlink_frame *frame;

    frame = mavlink_read(&dev);
    if (frame)
        print_frame(frame);

    frame = mavlink_read(&dev);
    if (frame)
        print_frame(frame);

    frame = mavlink_read(&dev);
    if (frame)
        print_frame(frame);

    frame = mavlink_read(&dev);
    if (frame)
        print_frame(frame);

    return 0;
}