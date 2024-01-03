

#include <iostream>
#include <string>

class Socket
{
public:


	bool tcpServer(std::string ip, int port);

	/// <summary>
	/// ͬ�� TCP socket �ͻ���
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	static bool tcpClientSync(std::string ip, int port);

	/// <summary>
	/// �첽 TCP socket �ͻ���
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	static bool tcpClientAsyn(std::string ip, int port);

	bool udpServer(std::string ip, int port);

	/// <summary>
	/// ͬ�� UDP socket �ͻ���
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	bool updClientSync(std::string ip, int port);

	/// <summary>
	/// �첽 UDP socket �ͻ���
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	bool updClientAsyn(std::string ip, int port);



	Socket() = default;
	~Socket() = default;

private:




};



































