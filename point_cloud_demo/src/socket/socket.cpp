#include "socket.h"

#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>



#define MAX_BUFFER_SIZE  1024


bool Socket::tcpServer(std::string ip, int port)
{

	return false;
}

// 同步
bool Socket::tcpClientSync(std::string ip, int port)
{
    bool status = false;
    // WSAStartup() 函数初始化了Winsock库
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }
    while (true)
    {
        // 创建套接字
        SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (clientSocket == INVALID_SOCKET) {
            std::cout << "Failed to create socket" << std::endl;
            WSACleanup();
            return false;
        }

        // 设置服务器信息
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) <= 0) {
            std::cout << "Invalid address/Address not supported" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }

        while (true) 
        {
            // 连接到服务器
            int is_con = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (is_con == SOCKET_ERROR) 
            {
                std::cout << "Connection failed" << std::endl;
                Sleep(5000); // 等待5秒后重新连接
                continue;
            }
            while (true)
            {
                char buffer[1024];
                memset(buffer, 0, sizeof(buffer));

                // 接收数据 (都是 <0 出错 =0 连接关闭 >0 接收到数据大小)
                size_t is_rec = recv(clientSocket, buffer, sizeof(buffer), 0);
                if ((int)is_rec == 0)
                {
                    std::cout << "Server disconnection and reconnection " << std::endl;
                    break;
                }
                else if((int)is_rec < 0)
                {
                    std::cout << "Failed to receive data" << std::endl;
                    continue;
                }
                else
                {
                    // 输出接收到的数据
                    std::cout << "Received data from server: " << buffer << std::endl;
                    Sleep(100); // 等待100毫秒后重新连接
                }
            }
            Sleep(100); // 跳出循环关闭socket 然后从新建立socket从新连接
            break;
        }

        closesocket(clientSocket);
        Sleep(3000); // 等待100毫秒后重新连接

    }

    WSACleanup();
    return true;
}


bool reconnect = false;

void receiveData(SOCKET clientSocket)
{
    while (true)
    {
        char buffer[1024];
        memset(buffer, 0, sizeof(buffer));

        int bytesRead = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);

        if (bytesRead == SOCKET_ERROR)
        {
            std::cout << "Failed to receive data: " << WSAGetLastError() << std::endl;
            reconnect = true;
            return;
        }
        if (bytesRead == 0)
        {
            std::cout << "Server disconnected." << std::endl;
            reconnect = false;
            return;
        }

        std::string receivedData(buffer, bytesRead);
        std::cout << "Received data: " << receivedData << std::endl;
    }
}

bool Socket::tcpClientAsyn(std::string ip, int port)
{
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cout << "Failed to initialize winsock." << std::endl;
        return false;
    }

    SOCKET clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == INVALID_SOCKET)
    {
        std::cout << "Failed to create socket: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return false;
    }
    while (true)
    {
        sockaddr_in serverAddress;
        serverAddress.sin_family = AF_INET;
        serverAddress.sin_port = htons(port);
        if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) != 1)
        {
            std::cout << "Invalid server IP address." << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }

        while (true)
        {
            int is_con = connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress));
            if (is_con == SOCKET_ERROR)
            {
                std::cout << "Failed to connect to the server: " << WSAGetLastError() << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待3秒再重新连接
                continue;
            }

            reconnect = false;
            std::thread receiveThread(receiveData, clientSocket);

            while (true)
            {
                if (reconnect)
                {
                    receiveThread.join();
                    closesocket(clientSocket);
                    break;
                }

                // 执行其他工作
                // ...
                std::cout << "a";

                std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待一段时间再继续接收数据
            }
        }

        closesocket(clientSocket);
    }

    WSACleanup();
    return true;
}







bool Socket::udpServer(std::string ip, int port)
{

	return false;
}

// 同步
bool Socket::updClientSync(std::string ip, int port)
{

	return false;
}

// 异步
bool Socket::updClientAsyn(std::string ip, int port)
{

	return false;
}




//Socket::Socket()
//{
//}
//
//Socket::~Socket()
//{
//}