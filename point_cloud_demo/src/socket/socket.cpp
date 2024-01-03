#include "socket.h"

#include <cstring>
#include <winsock2.h>
#include <ws2tcpip.h>

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
            if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
                std::cout << "Connection failed" << std::endl;
                Sleep(5000); // 等待5秒后重新连接
                continue;
            }
            while (true)
            {
                char buffer[1024];
                memset(buffer, 0, sizeof(buffer));

                // 接收数据
                if (recv(clientSocket, buffer, sizeof(buffer), 0) == SOCKET_ERROR) 
                {
                    std::cout << "Failed to receive data" << std::endl;
                    continue;
                }
                // 输出接收到的数据
                std::cout << "Received data from server: " << buffer << std::endl;
                Sleep(100); // 等待100毫秒后重新连接
            }
            // 关闭套接字
            Sleep(100); // 等待100毫秒后重新连接
            continue;
        }

        closesocket(clientSocket);
        Sleep(3000); // 等待100毫秒后重新连接

    }

    WSACleanup();
    return true;
}

static const int MAX_BUFFER_SIZE = 1024;

static void CALLBACK asyncSocketCallback(DWORD error, DWORD transferredBytes, LPWSAOVERLAPPED overlapped, DWORD flags)
{
    if (error == 0) {
        // 异步操作成功完成
        std::cout << "Async operation completed successfully" << std::endl;
    }
    else {
        // 异步操作出错
        std::cerr << "Async operation failed with error: " << error << std::endl;
    }
}

// 异步
bool Socket::tcpClientAsyn(std::string ip, int port)
{

    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cout << "Failed to initialize winsock" << std::endl;
        return false;
    }

    SOCKET clientSocket;
    sockaddr_in serverAddress;

    // 创建套接字
    clientSocket = WSASocket(AF_INET, SOCK_STREAM, 0, NULL, 0, WSA_FLAG_OVERLAPPED);
    if (clientSocket == INVALID_SOCKET) {
        std::cout << "Failed to create socket" << std::endl;
        WSACleanup();
        return false;
    }

    // 设置服务器信息
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &(serverAddress.sin_addr)) <= 0) {
        std::cout << "Invalid address/Address not supported" << std::endl;
        closesocket(clientSocket);
        WSACleanup();
        return false;
    }

    // 连接到服务器
    if (connect(clientSocket, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            std::cout << "Connection failed" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // 异步接收数据
    WSAOVERLAPPED overlapped;
    ZeroMemory(&overlapped, sizeof(WSAOVERLAPPED));
    char buffer[MAX_BUFFER_SIZE];
    WSABUF dataBuffer;
    dataBuffer.len = MAX_BUFFER_SIZE;
    dataBuffer.buf = buffer;
    overlapped.hEvent = buffer;
    DWORD receivedBytes;
    DWORD flags = 0;
    if (WSARecv(clientSocket, &dataBuffer, 1, &receivedBytes, &flags, &overlapped, asyncSocketCallback) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSA_IO_PENDING) {
            std::cout << "Failed to initiate async receive" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // 发送数据
    std::string message = "Hello, server!";
    if (send(clientSocket, message.c_str(), message.size(), 0) == SOCKET_ERROR) {
        if (WSAGetLastError() != WSAEWOULDBLOCK) {
            std::cout << "Failed to send data" << std::endl;
            closesocket(clientSocket);
            WSACleanup();
            return false;
        }
    }

    // 等待异步操作完成
    DWORD result;
    if (WSAGetOverlappedResult(clientSocket, &overlapped, &receivedBytes, TRUE, &flags) == FALSE) {
        std::cout << "Failed to get overlapped result" << std::endl;
    }

    // 输出接收到的数据
    std::cout << "Received data from server: " << buffer << std::endl;

    // 关闭套接字
    closesocket(clientSocket);
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