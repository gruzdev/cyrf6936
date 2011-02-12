/*
 * Example test program for CYRF6936 network driver.
 *
 * Copyright (c) 2011 Mikhail Gruzdev <michail.gruzdev@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/socket.h>
#include <netpacket/packet.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

static void err_exit(const char *msg)
{
	fflush(stdout);
	fprintf(stderr, "%s: %s\n", msg, strerror(errno));
	fflush(NULL);
	exit(EXIT_FAILURE);
}

int main()
{
	struct ifreq ifr;
	struct sockaddr_ll sa;
	char inbuf[100];
	size_t i, rxlen, rxnum = 0;
	int s;

	/* create socket */
	s = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (s < 0)
		err_exit("socket error");

	/* get interface index for binding */
	strcpy(ifr.ifr_name, "cwu0");
	if (ioctl(s, SIOCGIFINDEX, &ifr))
		err_exit("ioctl SIOCGIFINDEX error");

	/* fill sockaddr_ll struct to prepare binding */
	memset(&sa, 0, sizeof(struct sockaddr_ll));
	sa.sll_family = AF_PACKET;
	sa.sll_protocol = htons(ETH_P_ALL);
	sa.sll_ifindex = ifr.ifr_ifindex;

	/* bind socket to specific interface */
	if (bind(s, (struct sockaddr *)&sa, sizeof(struct sockaddr_ll)) < 0)
		err_exit("bind error");

	/* get data */
	for (;;) {
		/* receive */
		rxlen = recvfrom(s, inbuf, sizeof(inbuf), 0, NULL, NULL);
		/* dump packet contents */
		printf("#%08lu ", (unsigned long)++rxnum);
		for (i = 0; i < rxlen; i++)
			printf("%02x ", inbuf[i]);
		printf("\n");
	}
}
