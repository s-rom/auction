#pragma once
#include <string>
#include <cstring>

namespace Auction
{
	using std::string;
	using std::strcpy;
	using std::strlen;
	struct NetProfile;
}


struct Auction::NetProfile
{
	NetProfile(const string host_param, const string port_param)
	{
		host = new char[host_param.length() + 1];
		port = new char[port_param.length() + 1];
		
		strcpy(host, host_param.c_str());
		strcpy(port, port_param.c_str());
	}


	NetProfile(char* host_param, char* port_param)
	{
		size_t len_host = strlen(host_param);
		size_t len_port = strlen(port_param);

		host = new char[len_host + 1];
		port = new char[len_port + 1];

		strcpy(host, host_param);
		strcpy(port, port_param);
	}

	NetProfile(const NetProfile& other)
	{
		size_t len_host = strlen(other.host);
		size_t len_port = strlen(other.port);

		host = new char[len_host + 1];															
		port = new char[len_port + 1];

		strcpy(host, other.host);
		strcpy(port, other.port);
	}

	NetProfile operator=(const NetProfile& other)
	{
		return NetProfile(other);
	}


	~NetProfile()
	{
		delete[] host;
		delete[] port;
	}


	string to_string()
	{
		if (host == NULL || port == NULL)
			return "error";
		else
			return "host: " + string(host) + ",port: " + string(port);
	}

	char* host; // ip
	char* port; // Message system port



};