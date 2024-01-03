

#include <iostream>
#include <string>

class Socket
{
public:


	bool tcpServer(std::string ip, int port);

	/// <summary>
	/// 同步 TCP socket 客户端
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	static bool tcpClientSync(std::string ip, int port);

	/// <summary>
	/// 异步 TCP socket 客户端
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	static bool tcpClientAsyn(std::string ip, int port);

	bool udpServer(std::string ip, int port);

	/// <summary>
	/// 同步 UDP socket 客户端
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	bool updClientSync(std::string ip, int port);

	/// <summary>
	/// 异步 UDP socket 客户端
	/// </summary>
	/// <param name="ip"></param>
	/// <param name="port"></param>
	/// <returns></returns>
	bool updClientAsyn(std::string ip, int port);



	Socket() = default;
	~Socket() = default;

private:




};



































