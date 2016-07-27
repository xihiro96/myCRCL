//
// Created by mhendrix8 on 7/25/16.
//

#ifndef MY_CRCL_CRCLSOCKETINTERFACE_H
#define MY_CRCL_CRCLSOCKETINTERFACE_H

#include "crac/CRCLStatusClasses.hh"
#include "crac/crclCmds.hh"

#include "shared_ptr.h"

#include <boost/asio.hpp>

#include <string>
#include <memory>

namespace crcl
{

/**
 * @brief parse Parses a raw character buffer into a pointer to a CRCLStatusFile
 * @param xml_buf Pointer to raw character buffer containing message
 * @param len Length of message
 * @return
 */
    gtri::shared_ptr<CRCLStatusFile> parse(char *xml_buf, const int len);

/**
 * @brief Simple class for abstracting socket implementation details
 */
    class CRCLSocketInterface
    {
    public:
        CRCLSocketInterface(const std::string& host = "10.108.21.211", const int port = 64444);
        ~CRCLSocketInterface();

        void send(CrclCmdHeader* cmd);
        void send(const std::string& cmd);
        gtri::shared_ptr<CRCLStatusFile> receive(const size_t timeoutMillis = 100);

    protected:
        static const int buffLen = 2000;
        char buf[buffLen];

        boost::asio::io_service m_io;
        std::shared_ptr<boost::asio::ip::tcp::socket> mpSocket;
        std::shared_ptr<boost::asio::deadline_timer> mpTimer;
        //const size_t mTimeout;

        void connect(const std::string& host, const int port);

        void read_callback(bool& data_available, size_t& numRead, const boost::system::error_code& error, std::size_t bytes_transferred);
        void timer_callback(const boost::system::error_code& error);
    };

} // namespace crcl

#endif //MY_CRCL_CRCLSOCKETINTERFACE_H
