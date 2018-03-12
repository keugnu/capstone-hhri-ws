// Request.hpp

#ifndef __REQUEST_H_INCLUDED__
#define __REQUEST_H_INCLUDED__

#include <vector>

class Request {
    private:
        char _id;
        std::string _type;
        bool _status;
        int _size;
        int _expected;
    public:
        int num_attempts;
        std::vector<signed char> data;
    
        Request(const std::vector<signed char>, int);
        char get_id();
        std::string get_type();
        bool get_status();
        void set_status(bool);
        int get_size();
        int get_expected();
};

Request::Request(const std::vector<signed char> req, int num_bytes) {
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

int Request::get_size() { return _size; }
int Request::get_expected() { return _expected; }
bool Request::get_status() { return _status; }
char Request::get_id() { return _id; }
std::string Request::get_type() { return _type; }
void Request::set_status(bool status) { _status = status; }


#endif
