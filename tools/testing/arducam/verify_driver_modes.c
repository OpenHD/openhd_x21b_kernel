/* Exercise the real subdevice API without newer CLIENT_CAP ioctls. */
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int device;

static void format(unsigned int width, unsigned int height)
{
    struct v4l2_subdev_format fmt = {0};
    fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    fmt.format.width = width;
    fmt.format.height = height;
    fmt.format.code = MEDIA_BUS_FMT_UYVY8_1X16;
    assert(ioctl(device, VIDIOC_SUBDEV_S_FMT, &fmt) == 0);
    assert(fmt.format.width == width && fmt.format.height == height);
}

static void fps(unsigned int request, unsigned int expected)
{
    struct v4l2_subdev_frame_interval interval = {0};
    interval.interval.numerator = 1;
    interval.interval.denominator = request;
    assert(ioctl(device, VIDIOC_SUBDEV_S_FRAME_INTERVAL, &interval) == 0);
    assert(interval.interval.numerator == 1 && interval.interval.denominator == expected);
    memset(&interval, 0, sizeof(interval));
    assert(ioctl(device, VIDIOC_SUBDEV_G_FRAME_INTERVAL, &interval) == 0);
    assert(interval.interval.numerator == 1 && interval.interval.denominator == expected);
    printf("Requested %u fps -> driver interval 1/%u\n", request, expected);
}

int main(int argc, char **argv)
{
    unsigned int size, index;
    static const unsigned int rates[] = {30,60,120};
    struct v4l2_subdev_frame_interval_enum entry = {0};
    struct v4l2_subdev_frame_interval interval = {0};
    device = open(argc > 1 ? argv[1] : "/dev/v4l-subdev2", O_RDWR);
    if (device < 0) { perror("subdevice"); return 1; }
    for (size = 0; size < 2; size++) {
        for (index = 0; index < (size ? 2u : 3u); index++) {
            memset(&entry, 0, sizeof(entry));
            entry.which = V4L2_SUBDEV_FORMAT_ACTIVE;
            entry.code = MEDIA_BUS_FMT_UYVY8_1X16;
            entry.width = size ? 1920 : 1280;
            entry.height = size ? 1080 : 720;
            entry.index = index;
            assert(ioctl(device, VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL, &entry) == 0);
            assert(entry.interval.numerator == 1 && entry.interval.denominator == rates[index]);
            printf("Enumerated %ux%u @ %u fps\n",entry.width,entry.height,rates[index]);
        }
        entry.index++;
        errno = 0;
        assert(ioctl(device, VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL, &entry) == -1 && errno == EINVAL);
    }
    entry.pad = 1;
    assert(ioctl(device, VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL, &entry) == -1 && errno == EINVAL);
    entry.pad = 0; entry.width = 640; entry.height = 480; entry.index = 0;
    assert(ioctl(device, VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL, &entry) == -1 && errno == EINVAL);
    interval.interval.denominator = 60;
    assert(ioctl(device, VIDIOC_SUBDEV_S_FRAME_INTERVAL, &interval) == -1 && errno == EINVAL);
    format(1280,720);
    fps(120,120);
    format(1920,1080);
    fps(120,60);
    fps(30,30);
    fps(60,60);
    close(device);
    puts("PASS: nominal mode enumeration, invalid requests, 1080p FPS limit and cached intervals");
    return 0;
}
