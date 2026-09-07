/* SPDX-License-Identifier: GPL-2.0-only */
#define _GNU_SOURCE
#include <assert.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

static int mock_ioctl(int fd, unsigned long request, ...);
static ssize_t mock_sendmsg(int fd, const struct msghdr *msg, int flags);
static ssize_t mock_sendto(int fd, const void *data, size_t len, int flags,
			   const struct sockaddr *addr, socklen_t addrlen);
#define ioctl mock_ioctl
#define sendmsg mock_sendmsg
#define sendto mock_sendto
#include "../remote.c"
#undef ioctl
#undef sendmsg
#undef sendto

struct usteer_config config;
static unsigned int mtu_queries, sends, errors;
static bool fail_mtu;

void debug_msg(int level, const char *func, int line, const char *format, ...)
{
	errors++;
}

static int mock_ioctl(int fd, unsigned long request, ...)
{
	struct ifreq *ifr;
	va_list ap;

	assert(fd == 0 && request == SIOCGIFMTU);
	mtu_queries++;
	if (fail_mtu) {
		errno = ENODEV;
		return -1;
	}
	va_start(ap, request);
	ifr = va_arg(ap, struct ifreq *);
	ifr->ifr_mtu = 1500;
	va_end(ap);
	return 0;
}

static ssize_t mock_sendmsg(int fd, const struct msghdr *msg, int flags)
{
	assert(fd == 0 && msg->msg_iovlen == 1);
	sends++;
	return msg->msg_iov[0].iov_len;
}

static ssize_t mock_sendto(int fd, const void *data, size_t len, int flags,
			   const struct sockaddr *addr, socklen_t addrlen)
{
	assert(0);
	return -1;
}

int main(void)
{
	struct interface iface = {};

	iface.node.avl.key = "test0";
	blob_buf_init(&buf, 0);
	blob_put_int32(&buf, APMSG_ID, 1);
	blob_put_int32(&buf, APMSG_SEQ, 1);
	blob_put(&buf, APMSG_NODES, NULL, 0);
	assert(remote_fd.fd == -1 && !remote_fd.registered);
	interface_send_msg(&iface, buf.head);
	assert(!mtu_queries && !sends && !errors);
	/* Descriptor zero is valid after registration, but not before it. */
	remote_fd.fd = 0;
	interface_send_msg(&iface, buf.head);
	assert(!mtu_queries && !sends && !errors);
	remote_fd.registered = true;
	interface_send_msg(&iface, buf.head);
	assert(mtu_queries == 1 && sends == 1 && !errors);
	fail_mtu = true;
	interface_send_msg(&iface, buf.head);
	assert(mtu_queries == 2 && sends == 1 && errors == 1);
	remote_fd.registered = false;
	interface_send_msg(&iface, buf.head);
	assert(mtu_queries == 2 && sends == 1 && errors == 1);
	blob_buf_free(&buf);
	puts("remote-socket: all tests passed");
	return 0;
}
