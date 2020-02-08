#ifndef NET_PROFILE
#define NET_PROFILE

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

	/**
	 * Default constructor. 
	 * host and port are set to nullptr
	 */
	NetProfile()
	{
		host = nullptr;
		port = nullptr;
	}

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

	NetProfile(const char* host_param, const char* port_param)
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

	NetProfile& operator=(const NetProfile& other)
	{
		// self assignation
		if (this == &other) return *this;

		if (this->host) delete [] host;
		if (this->port) delete [] port;

		size_t len_host = strlen(other.host);
		size_t len_port = strlen(other.port);

		host = new char[len_host + 1];															
		port = new char[len_port + 1];

		strcpy(this->host, other.host);
		strcpy(this->port, other.port);

		return *this;
	}


	~NetProfile()
	{
		delete[] host;
		delete[] port;
	}


	bool operator==(const NetProfile & other)
	{
		int eq_host = strcmp(host,other.host);
		int eq_port = strcmp(port,other.port);

		return eq_host == 0 && eq_port == 0;
	}

	string to_string()
	{
		if (host == NULL || port == NULL)
			return "error";
		else
			return "host: " + string(host) + ", port: " + string(port);
	}

	char* host; // ip
	char* port; // Message system port



};

#endif