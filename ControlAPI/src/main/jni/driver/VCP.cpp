/*
 * VCP.cpp
 *
 *  Created on: Aug 21, 2014
 *      Author: asus
 */

#include <stdio.h>
//#include </home/asus/android-sdk/android-ndk-r10/toolchains/x86-4.8/prebuilt/linux-x86/lib/gcc/i686-linux-android/4.8/include-fixed/stdio.h>

#include <cstdlib>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/time.h>
#include <sys/ioctl.h>
//#include "/usr/include/linux/serial.h"
#include "../include/VCP.h"

#include <android/log.h>


//for Ubuntu
#include "/usr/src/linux-headers-3.16.0-49/include/uapi/linux/tty_flags.h"
//for Mint
//#include "/usr/src/linux-headers-3.13.0-24/include/uapi/linux/tty_flags.h"

#include "../include/DebugMessage.h"
#define LOG_TAG "VCPdebug"


bool PortIsOpen;

namespace asusbot {

	VCP::VCP() {
	}

	void VCP::SetPortName(const char *name) {

		strcpy(m_PortName, name);
	}

	bool VCP::OpenPort() {

		struct termios newtio;
		//struct serial_struct serinfo;

		ClosePort();

		m_Socket_fd = open(m_PortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (m_Socket_fd < 0) {

			LOGE("Open port fail...!");
			ClosePort();
			PortIsOpen = false;
			return false;
		}

		memset(&newtio, 0, sizeof(newtio));
		newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
		newtio.c_iflag = IGNPAR;
		newtio.c_oflag = 0;
		newtio.c_lflag = 0;
		newtio.c_cc[VTIME] = 0;
		newtio.c_cc[VMIN] = 0;

		//newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
		//newtio.c_cflag &= ~(PARENB | PARODD);

		tcflush(m_Socket_fd, TCIFLUSH);
		tcsetattr(m_Socket_fd, TCSANOW, &newtio);


		LOGI("COM port setting success!");

		PortIsOpen = true;
		return true;
	}

	VCP::~VCP() {
		// TODO Auto-generated destructor stub
		if(PortIsOpen == true)
		{
			ClosePort();
			PortIsOpen = false;
			LOGD("~VCP()");
		}
	}

	void VCP::Initial(const char *name) {

		PortIsOpen = false;
		SetPortName(name);
		LOGI("Initial to open COM port %s.", name);
		OpenPort();
	}

	int VCP::WritePort(unsigned char *packet, int numPacket) {
		int len = numPacket;

		#ifdef VIRTRUAL_COMPORT
		LOGD("VCPdebug Write--> ");
		for (int i = 0; i < len; i++)
			LOGD(" %x ", packet[i]);
		#endif

		return write(m_Socket_fd, packet, numPacket);
	}

	int VCP::ReadPort(unsigned char *Buffer, int len) {

		int bytesRcvd = 0;

		bytesRcvd = read(m_Socket_fd, Buffer, len);

		return bytesRcvd;

	}

	void VCP::ClosePort() {

		if (m_Socket_fd != -1) {
			LOGI("ClosePort()");
			close(m_Socket_fd);
		}

		m_Socket_fd = -1;
	}

	void VCP::bolcking(int should_block) {
		struct termios tty;
		memset(&tty, 0, sizeof tty);


		if (tcgetattr(m_Socket_fd, &tty) != 0) {
			LOGE("error %d from tggetattr", errno);
			return;
		}

		tty.c_cc[VMIN] = should_block ? 1 : 0;
		tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

		if (tcsetattr(m_Socket_fd, TCSANOW, &tty) != 0)
			LOGE("error %d setting term attributes", errno);
	}
}
