/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file spi_core.c
 * @author Nations
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#include <spi.h>

extern rt_err_t rt_spi_bus_device_init(struct rt_spi_bus *bus, const char *name);
extern rt_err_t rt_spidev_device_init(struct rt_spi_device *dev, const char *name);

rt_err_t rt_spi_bus_register(struct rt_spi_bus       *bus,
                             const char              *name,
                             const struct rt_spi_ops *ops)
{
    rt_err_t result;

    result = rt_spi_bus_device_init(bus, name);
    if (result != RT_EOK)
        return result;

    /* initialize mutex lock */
    rt_mutex_init(&(bus->lock), name, RT_IPC_FLAG_FIFO);
    /* set ops */
    bus->ops = ops;
    /* initialize owner */
    bus->owner = RT_NULL;
    /* set bus mode */
    bus->mode = RT_SPI_BUS_MODE_SPI;

    return RT_EOK;
}

rt_err_t rt_spi_bus_attach_device(struct rt_spi_device *device,
                                  const char           *name,
                                  const char           *bus_name,
                                  void                 *user_data)
{
    rt_err_t result;
    rt_device_t bus;

    /* get physical spi bus */
    bus = rt_device_find(bus_name);
    if (bus != RT_NULL && bus->type == RT_Device_Class_SPIBUS)
    {
        device->bus = (struct rt_spi_bus *)bus;

        /* initialize spidev device */
        result = rt_spidev_device_init(device, name);
        if (result != RT_EOK)
            return result;

        rt_memset(&device->config, 0, sizeof(device->config));
        device->parent.user_data = user_data;

        return RT_EOK;
    }

    /* not found the host bus */
    return -RT_ERROR;
}

rt_err_t rt_spi_configure(struct rt_spi_device        *device,
                          struct rt_spi_configuration *cfg)
{
    rt_err_t result;

    RT_ASSERT(device != RT_NULL);

    /* set configuration */
    device->config.data_width = cfg->data_width;
    device->config.mode       = cfg->mode & RT_SPI_MODE_MASK ;
    device->config.max_hz     = cfg->max_hz ;

    if (device->bus != RT_NULL)
    {
        result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
        if (result == RT_EOK)
        {
            if (device->bus->owner == device)
            {
                device->bus->ops->configure(device, &device->config);
            }
            /* release lock */
            rt_mutex_release(&(device->bus->lock));
        }
    }

    return RT_EOK;
}

rt_err_t rt_spi_send_then_send(struct rt_spi_device *device,
                               const void           *send_buf1,
                               rt_size_t             send_length1,
                               const void           *send_buf2,
                               rt_size_t             send_length2)
{
    rt_err_t result;
    struct rt_spi_message message;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        if (device->bus->owner != device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = device->bus->ops->configure(device, &device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                device->bus->owner = device;
            }
            else
            {
                /* configure SPI bus failed */
                result = -RT_EIO;
                goto __exit;
            }
        }

        /* send data1 */
        message.send_buf   = send_buf1;
        message.recv_buf   = RT_NULL;
        message.length     = send_length1;
        message.cs_take    = 1;
        message.cs_release = 0;
        message.next       = RT_NULL;

        result = device->bus->ops->xfer(device, &message);
        if (result == 0)
        {
            result = -RT_EIO;
            goto __exit;
        }

        /* send data2 */
        message.send_buf   = send_buf2;
        message.recv_buf   = RT_NULL;
        message.length     = send_length2;
        message.cs_take    = 0;
        message.cs_release = 1;
        message.next       = RT_NULL;

        result = device->bus->ops->xfer(device, &message);
        if (result == 0)
        {
            result = -RT_EIO;
            goto __exit;
        }

        result = RT_EOK;
    }
    else
    {
        return -RT_EIO;
    }

__exit:
    rt_mutex_release(&(device->bus->lock));

    return result;
}

rt_err_t rt_spi_send_then_recv(struct rt_spi_device *device,
                               const void           *send_buf,
                               rt_size_t             send_length,
                               void                 *recv_buf,
                               rt_size_t             recv_length)
{
    rt_err_t result;
    struct rt_spi_message message;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        if (device->bus->owner != device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = device->bus->ops->configure(device, &device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                device->bus->owner = device;
            }
            else
            {
                /* configure SPI bus failed */
                result = -RT_EIO;
                goto __exit;
            }
        }

        /* send data */
        message.send_buf   = send_buf;
        message.recv_buf   = RT_NULL;
        message.length     = send_length;
        message.cs_take    = 1;
        message.cs_release = 0;
        message.next       = RT_NULL;

        result = device->bus->ops->xfer(device, &message);
        if (result == 0)
        {
            result = -RT_EIO;
            goto __exit;
        }

        /* recv data */
        message.send_buf   = RT_NULL;
        message.recv_buf   = recv_buf;
        message.length     = recv_length;
        message.cs_take    = 0;
        message.cs_release = 1;
        message.next       = RT_NULL;

        result = device->bus->ops->xfer(device, &message);
        if (result == 0)
        {
            result = -RT_EIO;
            goto __exit;
        }

        result = RT_EOK;
    }
    else
    {
        return -RT_EIO;
    }

__exit:
    rt_mutex_release(&(device->bus->lock));

    return result;
}

rt_size_t rt_spi_transfer(struct rt_spi_device *device,
                          const void           *send_buf,
                          void                 *recv_buf,
                          rt_size_t             length)
{
    rt_err_t result;
    struct rt_spi_message message;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
    if (result == RT_EOK)
    {
        if (device->bus->owner != device)
        {
            /* not the same owner as current, re-configure SPI bus */
            result = device->bus->ops->configure(device, &device->config);
            if (result == RT_EOK)
            {
                /* set SPI bus owner */
                device->bus->owner = device;
            }
            else
            {
                /* configure SPI bus failed */
                rt_set_errno(-RT_EIO);
                result = 0;
                goto __exit;
            }
        }

        /* initial message */
        message.send_buf   = send_buf;
        message.recv_buf   = recv_buf;
        message.length     = length;
        message.cs_take    = 1;
        message.cs_release = 1;
        message.next       = RT_NULL;

        /* transfer message */
        result = device->bus->ops->xfer(device, &message);
        if (result == 0)
        {
            rt_set_errno(-RT_EIO);
            goto __exit;
        }
    }
    else
    {
        rt_set_errno(-RT_EIO);

        return 0;
    }

__exit:
    rt_mutex_release(&(device->bus->lock));

    return result;
}

struct rt_spi_message *rt_spi_transfer_message(struct rt_spi_device  *device,
                                               struct rt_spi_message *message)
{
    rt_err_t result;
    struct rt_spi_message *index;

    RT_ASSERT(device != RT_NULL);

    /* get first message */
    index = message;
    if (index == RT_NULL)
        return index;

    result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(-RT_EBUSY);

        return index;
    }

    /* reset errno */
    rt_set_errno(RT_EOK);

    /* configure SPI bus */
    if (device->bus->owner != device)
    {
        /* not the same owner as current, re-configure SPI bus */
        result = device->bus->ops->configure(device, &device->config);
        if (result == RT_EOK)
        {
            /* set SPI bus owner */
            device->bus->owner = device;
        }
        else
        {
            /* configure SPI bus failed */
            rt_set_errno(-RT_EIO);
            goto __exit;
        }
    }

    /* transmit each SPI message */
    while (index != RT_NULL)
    {
        /* transmit SPI message */
        result = device->bus->ops->xfer(device, index);
        if (result == 0)
        {
            rt_set_errno(-RT_EIO);
            break;
        }

        index = index->next;
    }

__exit:
    /* release bus lock */
    rt_mutex_release(&(device->bus->lock));

    return index;
}

rt_err_t rt_spi_take_bus(struct rt_spi_device *device)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    result = rt_mutex_take(&(device->bus->lock), RT_WAITING_FOREVER);
    if (result != RT_EOK)
    {
        rt_set_errno(-RT_EBUSY);

        return -RT_EBUSY;
    }

    /* reset errno */
    rt_set_errno(RT_EOK);

    /* configure SPI bus */
    if (device->bus->owner != device)
    {
        /* not the same owner as current, re-configure SPI bus */
        result = device->bus->ops->configure(device, &device->config);
        if (result == RT_EOK)
        {
            /* set SPI bus owner */
            device->bus->owner = device;
        }
        else
        {
            /* configure SPI bus failed */
            rt_set_errno(-RT_EIO);
            /* release lock */
            rt_mutex_release(&(device->bus->lock));

            return -RT_EIO;
        }
    }

    return result;
}

rt_err_t rt_spi_release_bus(struct rt_spi_device *device)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->owner == device);

    /* release lock */
    rt_mutex_release(&(device->bus->lock));

    return RT_EOK;
}

rt_err_t rt_spi_take(struct rt_spi_device *device)
{
    rt_err_t result;
    struct rt_spi_message message;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    rt_memset(&message, 0, sizeof(message));
    message.cs_take = 1;

    result = device->bus->ops->xfer(device, &message);

    return result;
}

rt_err_t rt_spi_release(struct rt_spi_device *device)
{
    rt_err_t result;
    struct rt_spi_message message;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);

    rt_memset(&message, 0, sizeof(message));
    message.cs_release = 1;

    result = device->bus->ops->xfer(device, &message);

    return result;
}
