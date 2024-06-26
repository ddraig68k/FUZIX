#include <kernel.h>
#include <kdata.h>
#include <input.h>
#include <devinput.h>

static char buf[32];
static struct s_queue kqueue = {
 buf, buf, buf, sizeof(buf), 0, sizeof(buf) / 2
};

/* Queue a character to the input device */
void queue_input(uint8_t c)
{
    insq(&kqueue, c);
    wakeup(&kqueue);
}

int plt_input_read(uint8_t *slot)
{
    uint_fast8_t r, k;
    if (remq(&kqueue, &r)) {
        remq(&kqueue, &k);
	*slot++ = KEYPRESS_CODE | r;
	*slot++ = k;
	return 2;
    }
    udata.u_error = EAGAIN;
    return -1;
}

void plt_input_wait(void)
{
    psleep(&kqueue);
}

int plt_input_write(uint_fast8_t flag)
{
    flag;
    udata.u_error = EINVAL;
    return -1;
}

void poll_input(void)
{
}
