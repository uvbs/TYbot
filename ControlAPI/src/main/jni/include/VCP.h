/*
 * VCP.h
 *
 *  Created on: Aug 21, 2014
 *      Author: asus
 */

#ifndef VCP_H_
#define VCP_H_


namespace asusbot {

	class VCP {
	public:

		int m_Socket_fd;
		char m_PortName[20];


		VCP();
		virtual ~VCP();

		void Initial(const char *name);
		bool OpenPort();
		void ClosePort();
		void bolcking(int should_block);
		void SetPortName(const char *name);
		int ReadPort(unsigned char *Buffer, int len);
		int WritePort(unsigned char *packet, int numPacket);
	};
}
#endif /* VCP_H_ */
