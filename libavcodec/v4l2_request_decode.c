/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "config.h"

#include <poll.h>
#include <sys/ioctl.h>

#include "decode.h"
#include "v4l2_request_internal.h"

#define INPUT_BUFFER_PADDING_SIZE   (AV_INPUT_BUFFER_PADDING_SIZE * 4)

uint64_t ff_v4l2_request_get_capture_timestamp(AVFrame *frame)
{
    V4L2RequestFrameDescriptor *desc = v4l2_request_framedesc(frame);

    /*
     * The capture buffer index is used as a base for V4L2 frame reference.
     * This works because frames are decoded into a capture buffer that is
     * closely tied to an AVFrame.
     */
    return desc ? v4l2_timeval_to_ns(&desc->capture.buffer.timestamp) : 0;
}

static int v4l2_request_queue_buffer(V4L2RequestContext *ctx, int request_fd,
                                     V4L2RequestBuffer *buf, uint32_t flags)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .index = buf->index,
        .type = buf->buffer.type,
        .memory = buf->buffer.memory,
        .timestamp = buf->buffer.timestamp,
        .bytesused = buf->used,
        .request_fd = request_fd,
        .flags = ((request_fd >= 0) ? V4L2_BUF_FLAG_REQUEST_FD : 0) | flags,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
        planes[0].bytesused = buf->used;
        buffer.bytesused = 0;
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    if (!V4L2_TYPE_IS_OUTPUT(buffer.type))
        ctx = v4l2_request_capture_context(ctx);
    // Queue the buffer
    if (ioctl(ctx->video_fd, VIDIOC_QBUF, &buffer) < 0)
        return AVERROR(errno);

    // Mark the buffer as queued
    if (V4L2_TYPE_IS_OUTPUT(buffer.type))
        atomic_fetch_or(&ctx->queued_output, 1U << buffer.index);
    else {
        atomic_fetch_and(&ctx->capture_errors, ~(UINT64_C(1) << buffer.index));
        atomic_fetch_or(&ctx->queued_capture, UINT64_C(1) << buffer.index);
    }

    return 0;
}

static int v4l2_request_dequeue_buffer(V4L2RequestContext *ctx,
                                       enum v4l2_buf_type type)
{
    struct v4l2_plane planes[1] = {};
    struct v4l2_buffer buffer = {
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
        buffer.length = 1;
        buffer.m.planes = planes;
    }

    if (!V4L2_TYPE_IS_OUTPUT(type))
        ctx = v4l2_request_capture_context(ctx);
    // Dequeue next completed buffer
    if (ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer) < 0)
        return AVERROR(errno);

    /*
     * The driver marks a capture buffer it could not fill with
     * V4L2_BUF_FLAG_ERROR.  The m2m path in v4l2_buffers.c reports this as a
     * decode error; the request path did not look at the flag at all, so a
     * driver-side failure arrived at the caller as black pixels or as
     * uninitialised memory with a successful decode count.  Measured on
     * IP1001: the histb-vdec driver rejected every frame of a 3840x2160
     * H.264 stream with -ENOMEM, marked each job VB2_BUF_STATE_ERROR, and
     * ffmpeg still reported five decoded frames whose contents were entirely
     * zero.  Log it here so the failure is at least visible.
     */
    if (!V4L2_TYPE_IS_OUTPUT(buffer.type) &&
        (buffer.flags & V4L2_BUF_FLAG_ERROR)) {
        atomic_fetch_or(&ctx->capture_errors, UINT64_C(1) << buffer.index);
        av_log(ctx, AV_LOG_ERROR,
               "capture buffer %u carries V4L2_BUF_FLAG_ERROR; the driver did not "
               "produce this frame\n", buffer.index);
    }

    // Mark the buffer as dequeued
    if (V4L2_TYPE_IS_OUTPUT(buffer.type))
        atomic_fetch_and(&ctx->queued_output, ~(1 << buffer.index));
    else
        atomic_fetch_and(&ctx->queued_capture, ~(UINT64_C(1) << buffer.index));

    return 0;
}

static inline int v4l2_request_dequeue_completed_buffers(V4L2RequestContext *ctx,
                                                         enum v4l2_buf_type type)
{
    int ret;

    do {
        ret = v4l2_request_dequeue_buffer(ctx, type);
    } while (!ret);

    return ret;
}

int ff_v4l2_request_wait_capture(V4L2RequestContext *ctx,
                               V4L2RequestBuffer *capture, bool check_error)
{
    struct pollfd pollfd = { .events = POLLIN };

    ctx = v4l2_request_capture_context(ctx);
    pollfd.fd = ctx->video_fd;

    ff_mutex_lock(&ctx->mutex);

    // Dequeue all completed capture buffers
    if (atomic_load(&ctx->queued_capture))
        v4l2_request_dequeue_completed_buffers(ctx, ctx->format.type);

    // Wait on the specific capture buffer, when needed
    while (atomic_load(&ctx->queued_capture) & (UINT64_C(1) << capture->index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            goto fail;

        ret = v4l2_request_dequeue_buffer(ctx, ctx->format.type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            goto fail;
    }

    ff_mutex_unlock(&ctx->mutex);
    return (check_error && (atomic_load(&ctx->capture_errors) &
            (UINT64_C(1) << capture->index))) ? AVERROR_INVALIDDATA : 0;

fail:
    ff_mutex_unlock(&ctx->mutex);
    av_log(ctx, AV_LOG_ERROR, "Failed waiting on capture buffer %d\n",
           capture->index);
    return AVERROR(EINVAL);
}

static int v4l2_request_wait_on_frame(void *opaque)
{
    V4L2RequestFrameDescriptor *desc = opaque;
    V4L2RequestContext *ctx;

    if (!desc || !desc->ctx)
        return AVERROR(EINVAL);

    ctx = desc->ctx;
    return ff_v4l2_request_wait_capture(ctx, &desc->capture, true);
}

static V4L2RequestBuffer *v4l2_request_next_output(V4L2RequestContext *ctx)
{
    int index;
    V4L2RequestBuffer *output;
    struct pollfd pollfd = {
        .fd = ctx->video_fd,
        .events = POLLOUT,
    };

    ff_mutex_lock(&ctx->mutex);

    // Use next output buffer in the circular queue
    index = atomic_load(&ctx->next_output);
    output = &ctx->output[index];
    atomic_store(&ctx->next_output, (index + 1) % FF_ARRAY_ELEMS(ctx->output));

    // Dequeue all completed output buffers
    if (atomic_load(&ctx->queued_output))
        v4l2_request_dequeue_completed_buffers(ctx, ctx->output_type);

    // Wait on the specific output buffer, when needed
    while (atomic_load(&ctx->queued_output) & (1 << output->index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            goto fail;

        ret = v4l2_request_dequeue_buffer(ctx, ctx->output_type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            goto fail;
    }

    ff_mutex_unlock(&ctx->mutex);

    // Reset used state
    output->used = 0;

    return output;

fail:
    ff_mutex_unlock(&ctx->mutex);
    av_log(ctx, AV_LOG_ERROR, "Failed waiting on output buffer %d\n",
           output->index);
    return NULL;
}

static int v4l2_request_wait_on_request(V4L2RequestContext *ctx,
                                        V4L2RequestBuffer *output)
{
    struct pollfd pollfd = {
        .fd = output->fd,
        .events = POLLPRI,
    };

    // Wait on the specific request to complete, when needed
    while (atomic_load(&ctx->queued_request) & (1 << output->index)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            break;

        // Mark request as dequeued
        if (pollfd.revents & (POLLPRI | POLLERR)) {
            atomic_fetch_and(&ctx->queued_request, ~(1 << output->index));
            break;
        }
    }

    // Reinit the request object
    if (ioctl(output->fd, MEDIA_REQUEST_IOC_REINIT, NULL) < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to reinit request object %d: %s (%d)\n",
               output->fd, strerror(errno), errno);
        return AVERROR(errno);
    }

    // Ensure request is marked as dequeued
    atomic_fetch_and(&ctx->queued_request, ~(1 << output->index));

    return 0;
}

int ff_v4l2_request_append_output(AVCodecContext *avctx,
                                  V4L2RequestPictureContext *pic,
                                  const uint8_t *data, uint32_t size)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    // Append data to output buffer and ensure there is enough space for padding
    if (pic->output->used + size + INPUT_BUFFER_PADDING_SIZE <= pic->output->size) {
        memcpy(pic->output->addr + pic->output->used, data, size);
        pic->output->used += size;
        return 0;
    } else {
        av_log(ctx, AV_LOG_ERROR,
               "Failed to append %u bytes data to output buffer %d (%u of %u used)\n",
               size, pic->output->index, pic->output->used, pic->output->size);
        return AVERROR(ENOMEM);
    }
}

static int v4l2_request_queue_decode(AVCodecContext *avctx,
                                     V4L2RequestPictureContext *pic,
                                     struct v4l2_ext_control *control, int count,
                                     bool first_slice, bool last_slice)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    uint32_t flags;
    int ret;

    if (first_slice) {
        /*
         * Wait on dequeue of the target capture buffer, when needed. Otherwise
         * V4L2 decoder may use a different capture buffer than hwaccel expects.
         *
         * Normally decoding has already completed when a capture buffer is
         * reused so this is more or less a no-op, however in some situations
         * FFmpeg may reuse an AVFrame early, i.e. when no output frame was
         * produced prior time, and a syncronization is necessary.
         */
        ret = ff_v4l2_request_wait_capture(ctx, pic->capture, false);
        if (ret < 0)
            return ret;
    }

    ff_mutex_lock(&ctx->mutex);

    /*
     * The output buffer tied to prior use of current request object can
     * independently be dequeued before the full decode request has been
     * completed. This may happen when a decoder use multi stage decoding,
     * e.g. rpivid. In such case we can start reusing the output buffer,
     * however we must wait on the prior request to fully complete before we
     * can reuse the request object, and a syncronization is necessary.
     */
    ret = v4l2_request_wait_on_request(ctx, pic->output);
    if (ret < 0)
        goto fail;

    /*
     * Dequeue any completed output buffers, this is strictly not necessary,
     * however if a syncronization was necessary for the capture and/or request
     * there is more than likely one or more output buffers that can be dequeued.
     */
    if (atomic_load(&ctx->queued_output))
        v4l2_request_dequeue_completed_buffers(ctx, ctx->output_type);

    // Set codec controls for current request
    ret = ff_v4l2_request_set_request_controls(ctx, pic->output->fd, control, count);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to set %d control(s) for request %d: %s (%d)\n",
               count, pic->output->fd, strerror(errno), errno);
        goto fail;
    }

    // Ensure there is zero padding at the end of bitstream data
    memset(pic->output->addr + pic->output->used, 0, INPUT_BUFFER_PADDING_SIZE);

    // Use timestamp of the capture buffer for V4L2 frame reference
    pic->output->buffer.timestamp = pic->capture->buffer.timestamp;

    /*
     * Queue the output buffer of current request. The capture buffer may be
     * hold by the V4L2 decoder unless this is the last slice of a frame.
     */
    flags = last_slice ? 0 : V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
    ret = v4l2_request_queue_buffer(ctx, pic->output->fd, pic->output, flags);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to queue output buffer %d for request %d: %s (%d)\n",
               pic->output->index, pic->output->fd, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    if (first_slice) {
        /*
         * Queue the target capture buffer, hwaccel expect and depend on that
         * this specific capture buffer will be used as decode target for
         * current request, otherwise frames may be output in wrong order or
         * wrong capture buffer could get used as a reference frame.
         */
        ret = v4l2_request_queue_buffer(ctx, -1, pic->capture, 0);
        if (ret < 0) {
            av_log(ctx, AV_LOG_ERROR, "Failed to queue capture buffer %d for request %d: %s (%d)\n",
                   pic->capture->index, pic->output->fd, strerror(errno), errno);
            ret = AVERROR(errno);
            goto fail;
        }
    }

    // Queue current request
    ret = ioctl(pic->output->fd, MEDIA_REQUEST_IOC_QUEUE, NULL);
    if (ret < 0) {
        av_log(ctx, AV_LOG_ERROR, "Failed to queue request object %d: %s (%d)\n",
               pic->output->fd, strerror(errno), errno);
        ret = AVERROR(errno);
        goto fail;
    }

    // Mark current request as queued
    atomic_fetch_or(&ctx->queued_request, 1 << pic->output->index);

    ret = 0;
fail:
    ff_mutex_unlock(&ctx->mutex);
    return ret;
}

int ff_v4l2_request_decode_slice(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count,
                                 bool first_slice, bool last_slice)
{
    /*
     * Fallback to queue each slice as a full frame when holding capture
     * buffers is not supported by the driver.
     */
    if ((pic->output->capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF) !=
         V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF)
        return v4l2_request_queue_decode(avctx, pic, control, count, true, true);

    return v4l2_request_queue_decode(avctx, pic, control, count,
                                     first_slice, last_slice);
}

int ff_v4l2_request_decode_frame(AVCodecContext *avctx,
                                 V4L2RequestPictureContext *pic,
                                 struct v4l2_ext_control *control, int count)
{
    return v4l2_request_queue_decode(avctx, pic, control, count, true, true);
}

static int v4l2_request_post_process(void *logctx, AVFrame *frame)
{
    V4L2RequestFrameDescriptor *desc = v4l2_request_framedesc(frame);
    FrameDecodeData *fdd = frame->private_ref;
    V4L2RequestContext *ctx = fdd->hwaccel_priv;
    bool defer = ctx->defer_capture_wait &&
        (!ctx->defer_capture_wait_auto ||
         (!(frame->flags & AV_FRAME_FLAG_INTERLACED) &&
          (frame->width > desc->capture.width ||
           frame->height > desc->capture.height)));
    int ret;

    /*
     * The normal path completes capture before publishing the frame.  The
     * opt-in pipeline path publishes the DRM frame as soon as its request has
     * been queued, then waits when hwdownload actually maps it.  This lets the
     * frame-thread parser queue the next request while the device finishes
     * VDH/VPSS, without exposing incomplete pixels to the CPU.
     */
    if (!defer) {
        ret = ff_v4l2_request_wait_capture(ctx, &desc->capture, true);
        if (ret < 0)
            return ret;
        desc->base.wait = NULL;
        desc->base.wait_opaque = NULL;
    }

    /*
     * ff_get_buffer() initially describes the coded picture.  A V4L2
     * post-processor may allocate a smaller capture buffer, so publish the
     * actual DMA buffer dimensions before filters or encoders consume it.
     */
    if (frame->width != desc->capture.width ||
        frame->height != desc->capture.height) {
        unsigned int visible_width = frame->width -
                                     frame->crop_left - frame->crop_right;
        unsigned int visible_height = frame->height -
                                      frame->crop_top - frame->crop_bottom;

        if (frame->sample_aspect_ratio.num > 0 &&
            visible_width && visible_height) {
            frame->sample_aspect_ratio = av_mul_q(frame->sample_aspect_ratio,
                av_div_q((AVRational) { visible_width, visible_height },
                         (AVRational) { desc->capture.width,
                                        desc->capture.height }));
        }
        frame->width = desc->capture.width;
        frame->height = desc->capture.height;
        frame->crop_left = frame->crop_right = 0;
        frame->crop_top = frame->crop_bottom = 0;
    }

    return 0;
}

int ff_v4l2_request_reset_picture(AVCodecContext *avctx, V4L2RequestPictureContext *pic)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);

    // Get and wait on next output buffer from circular queue
    pic->output = v4l2_request_next_output(ctx);
    if (!pic->output)
        return AVERROR(EINVAL);

    return 0;
}

int ff_v4l2_request_start_frame(AVCodecContext *avctx,
                                V4L2RequestPictureContext *pic,
                                AVFrame *frame)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    V4L2RequestFrameDescriptor *desc = v4l2_request_framedesc(frame);
    FrameDecodeData *fdd = frame->private_ref;
    int ret;

    // Get next output buffer from circular queue
    ret = ff_v4l2_request_reset_picture(avctx, pic);
    if (ret)
        return ret;

    // Ensure capture buffer is dequeued before reuse
    /* Reusing a completed failed buffer is valid: QBUF starts a new
     * payload lifetime and clears its old error. Consumers still fail. */
    ret = ff_v4l2_request_wait_capture(ctx, &desc->capture, false);
    if (ret)
        return ret;

    // Wait on capture buffer in post_process() before returning to application
    fdd->hwaccel_priv = ctx;
    fdd->post_process = v4l2_request_post_process;

    desc->ctx = v4l2_request_capture_context(ctx);
    desc->base.wait = v4l2_request_wait_on_frame;
    desc->base.wait_opaque = desc;

    // Capture buffer used for current frame
    pic->capture = &desc->capture;

    return 0;
}

void ff_v4l2_request_flush(AVCodecContext *avctx)
{
    V4L2RequestContext *ctx = v4l2_request_context(avctx);
    struct pollfd pollfd = {
        .fd = ctx->video_fd,
        .events = POLLOUT,
    };

    ff_mutex_lock(&ctx->mutex);

    // Dequeue all completed output buffers
    if (atomic_load(&ctx->queued_output))
        v4l2_request_dequeue_completed_buffers(ctx, ctx->output_type);

    // Wait on any remaining output buffer
    while (atomic_load(&ctx->queued_output)) {
        int ret = poll(&pollfd, 1, 2000);
        if (ret <= 0)
            break;

        ret = v4l2_request_dequeue_buffer(ctx, ctx->output_type);
        if (ret < 0 && ret != AVERROR(EAGAIN))
            break;
    }

    ff_mutex_unlock(&ctx->mutex);

    /* Capture/VPSS can finish after OUTPUT is returned. Keep its queue
     * state and fd valid for downstream frames through the final dequeue. */
    ctx = v4l2_request_capture_context(ctx);
    pollfd.fd = ctx->video_fd;
    pollfd.events = POLLIN;
    ff_mutex_lock(&ctx->mutex);
    while (atomic_load(&ctx->queued_capture)) {
        int ret = v4l2_request_dequeue_buffer(ctx, ctx->format.type);

        if (ret == AVERROR(EAGAIN)) {
            ret = poll(&pollfd, 1, 2000);
            if (ret > 0)
                continue;
        } else if (!ret) {
            continue;
        }
        /* Canceled or unfinished capture must never become valid pixels. */
        atomic_fetch_or(&ctx->capture_errors,
                        atomic_exchange(&ctx->queued_capture, 0));
        break;
    }
    ff_mutex_unlock(&ctx->mutex);
}
