#include "debug_log.h"
#include "bsp_uart_debug.h"
#include "debug_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

void DebugLog_Init(void)
{
    BspUartDebug_Init();
}

uint16_t DebugLog_Write(const uint8_t *data, uint16_t len)
{
    return BspUartDebug_Write(data, len);
}

void DebugLog_Print(const char *fmt, ...)
{
#if DEBUG_LOG_ENABLE
    char buf[DEBUG_LOG_LINE_SIZE];
    int len = 0;
    va_list ap;

    if (fmt == NULL)
    {
        return;
    }

    len = snprintf(buf, sizeof(buf), "%s", DEBUG_LOG_PREFIX);
    if (len < 0)
    {
        return;
    }

    va_start(ap, fmt);
    len += vsnprintf(&buf[len], sizeof(buf) - (size_t)len, fmt, ap);
    va_end(ap);

    if (len < 0)
    {
        return;
    }

    if ((size_t)len >= sizeof(buf))
    {
        len = (int)(sizeof(buf) - 1U);
        buf[len] = '\0';
    }

#if DEBUG_LOG_AUTO_CRLF
    if ((len + 2) < (int)sizeof(buf))
    {
        if ((len < 2) || !(buf[len - 2] == '\r' && buf[len - 1] == '\n'))
        {
            buf[len++] = '\r';
            buf[len++] = '\n';
            buf[len] = '\0';
        }
    }
#endif

    (void)BspUartDebug_Write((const uint8_t *)buf, (uint16_t)len);
#else
    (void)fmt;
#endif
}

uint16_t DebugLog_Read(uint8_t *data, uint16_t len)
{
    return BspUartDebug_Read(data, len);
}

uint16_t DebugLog_GetRxCount(void)
{
    return BspUartDebug_GetRxCount();
}

uint16_t DebugLog_GetDiscarded(void)
{
    return BspUartDebug_GetTxDiscarded();
}

void DebugLog_ResetDiscarded(void)
{
    BspUartDebug_ResetTxDiscarded();
}

