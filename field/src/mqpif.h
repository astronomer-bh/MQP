// Code from Nick Green, modified by Brian Healy

#ifndef mqpif_h_included
#define mqpif_h_included


//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
class MQPIf {
private:
	bool		connected;

	int			socket;
	int			ip;

	char		result[256];

public:
	MQPIf();
	~MQPIf();

	bool connect(const char * const host, const int port);
	void disconnect();

	bool sendOne(const void *msg, const int msg_len);

	const char * getResult()			{ return result; }
};


#endif	// mqpif_h_included
