// Request.hpp

#ifndef __REQUEST_H_INCLUDED__
#define __REQUEST_H_INCLUDED__

#include <vector>

/*  Class: Request
    attrs:
        _id: address of device
        _type: type of request (read, write, status)
        _status: true if request is fullfilled, else false
        _size: size of incoming request in bytes
        _expected: number of expected bytes returned from device for read request
        num_attempts: number of attempts the request was served
        data: request data vector
    members:
        get_id: gets address of device for the request
        get_type: gets the type of request
        get_status: gets the current status of the request
        get_size: gets the size of the request
        get_expected: gets the expected number of bytes that should be returned by the device
*/
class Request {
    private:
        uint8_t _id;
        std::string _type;
        bool _status;
        int _size;
        int _expected;
    public:
        int num_attempts;
        std::vector<uint8_t> data;
    
        Request(const std::vector<uint8_t>, int);
        uint8_t get_id();
        std::string get_type();
        bool get_status();
        void set_status(bool);
        int get_size();
        int get_expected();
};

/*  Constructor: Request
    desc: Default constructor for the Request class.
    inputs:
        req[]: frame containing the request content
                req[0]: type of request
                req[1]: address of the device
                req[2n]: register address
                req[2n+1]: data byte for write request on registers 2n
        num_bytes: size of req[]
    defaults:
        _status: false
        num_attempts: 0
*/
Request::Request(const std::vector<uint8_t> req, int num_bytes) {
    switch (req[0]) {
        case 0:
            _type = "status";
        break;
        case 1:
            _type = "read";
        break;
        case 2:
            _type = "write";
        break;
    }

    _id = req[1];
    _status = false;
    _size = num_bytes;
    num_attempts = 0;

    for (int i = 0; i < _size - 2; i++) { data.push_back(req[i + 2]); }

}

/* Request class function prototypes */
int Request::get_size() { return _size; }
int Request::get_expected() { return _expected; }
bool Request::get_status() { return _status; }
uint8_t Request::get_id() { return _id; }
std::string Request::get_type() { return _type; }
void Request::set_status(bool status) { _status = status; }

#endif
