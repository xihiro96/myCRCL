//
// Created by mhendrix8 on 7/25/16.
//
#include "CRCLSocketInterface.h"
#include "crac/ulapi.hh"

#include <boost/bind.hpp>

#include <iostream>

using boost::asio::ip::tcp;

// XML parser globals
extern int yysparse();
extern char *yysStringInputPointer;
extern char *yysStringInputEnd;
extern CRCLStatusFile *CRCLStatusTree; // NB: the parser creates a new() value for this each time.

namespace crcl
{

    gtri::shared_ptr<CRCLStatusFile> parse(char *xml_buf, const int len)
    {
        // Initialize variables for parsing using NIST parser
        yysStringInputPointer = xml_buf;
        yysStringInputEnd = &xml_buf[len];
        yysparse();

        return gtri::shared_ptr<CRCLStatusFile>(CRCLStatusTree);
    }

    CRCLSocketInterface::CRCLSocketInterface(const std::string& host, const int port)
    {
        mpTimer = std::make_shared<boost::asio::deadline_timer>(m_io);
        mpSocket = std::make_shared<tcp::socket>(m_io);
        connect(host, port);
    }

    void CRCLSocketInterface::connect(const std::string& host, const int port)
    {
        tcp::resolver resolver{m_io};
        tcp::resolver::iterator endpoint = resolver.resolve(tcp::resolver::query(tcp::v4(), host, std::to_string(port)));


        boost::system::error_code error = boost::asio::error::host_not_found;
        tcp::resolver::iterator end;
        while (error && endpoint != end)
        {
            mpSocket->close();
            mpSocket->connect(*endpoint++, error);
        }
        if (error) { throw boost::system::system_error(error); }
    }

    CRCLSocketInterface::~CRCLSocketInterface()
    {
        if (mpSocket->is_open())
        {
            mpSocket->shutdown(boost::asio::socket_base::shutdown_both);
            mpSocket->close();
        }
    }

    void CRCLSocketInterface::send(CrclCmdHeader *cmd)
    {
        send(cmd->getCmd().xmlCmd);
    }

    void CRCLSocketInterface::send(const std::string& cmd)
    {
        std::size_t numWritten = boost::asio::write(*mpSocket, boost::asio::buffer(cmd.c_str(), cmd.size()));
        assert(cmd.size() == numWritten);
    }

    gtri::shared_ptr<CRCLStatusFile> CRCLSocketInterface::receive(const size_t timeoutMillis)
    {
        gtri::shared_ptr<CRCLStatusFile> retVal = gtri::shared_ptr<CRCLStatusFile>();

        // Read async
        bool data_available = false;
        std::size_t bytesRead = 0;
        boost::asio::async_read(*mpSocket, boost::asio::buffer(&buf[0], buffLen), boost::asio::transfer_all(),
                                boost::bind(&CRCLSocketInterface::read_callback, this,
                                            boost::ref(data_available),
                                            boost::ref(bytesRead),
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));

        // Setup timer callback
        mpTimer->expires_from_now(boost::posix_time::milliseconds(timeoutMillis));
        mpTimer->async_wait(boost::bind(&CRCLSocketInterface::timer_callback, this,
                                        boost::asio::placeholders::error));

        m_io.run();
        m_io.reset();

        if (!data_available)
        {
            std::cerr << "No bytes read." << std::endl;
            return retVal;
        }

        // Attempt to parse
        try
        {
            retVal = parse(buf, bytesRead);
        }
        catch (std::exception e)
        {
            std::cerr << e.what() << std::endl;
        }

        return retVal;
    }


// Data was read or read was cancelled
    void CRCLSocketInterface::read_callback(bool& data_available, size_t& numRead, const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        numRead = bytes_transferred;
        if (!bytes_transferred || boost::asio::error::operation_aborted != error) // timeout triggers aborted
        {
            // No data was read!
            data_available = false;
            return;
        }

        mpTimer->cancel();  // will cause wait_callback to fire with an error
        data_available = true;
    }

// Timeout occurred or timer was cancelled
    void CRCLSocketInterface::timer_callback(const boost::system::error_code& error)
    {
        if (error)
        {
            // Data was read and this timeout was canceled
            return;
        }

        mpSocket->cancel();  // will cause read_callback to fire with an error
    }

    void checkType()
    {
        CRCLCommandInstanceFile* file;
        file->CRCLCommandInstance->CRCLCommand;
        MiddleCommandType* midCmd = dynamic_cast<MiddleCommandType*>(file->CRCLCommandInstance->CRCLCommand);
        if (nullptr != midCmd) {
            MoveToType *moveToCmd = dynamic_cast<MoveToType *>(midCmd);
            if (nullptr != moveToCmd) {
                std::cout << "This is a MoveToType" << std::endl;
                //return 0;
            }

            SetEndEffectorType *seeCmd = dynamic_cast<SetEndEffectorType *>(midCmd);
            if (nullptr != seeCmd) {
                std::cout << "This is a SetEndEffectorType" << std::endl;
                //return;
            }

            ActuateJointsType* actuateType = dynamic_cast<ActuateJointsType*>(midCmd);
            if (nullptr != actuateType)
            {
                std::cout << "This is an ActuateJointsType" << std::endl;

            }

            CloseToolChangerType* closeTool = dynamic_cast<CloseToolChangerType*>(midCmd);
            if (nullptr != closeTool)
            {
                std::cout << "This is a CloseToolChangerType" << std::endl;
            }

            ConfigureJointReportsType* configureJoint = dynamic_cast<ConfigureJointReportsType*>(midCmd);
            if (nullptr != configureJoint)
            {
                std::cout << "This is a ConfigureJointReportsType" << std::endl;
            }

            DwellType* dwellType = dynamic_cast<DwellType*>(midCmd);
            if (nullptr != dwellType)
            {
                std::cout << "This is a DwellType" << std::endl;
            }

            GetStatusType* getStatus = dynamic_cast<GetStatusType*>(midCmd);
            if (nullptr != getStatus)
            {
                std::cout << "This is a GetStatusType" << std::endl;
            }

            MessageType* message = dynamic_cast<MessageType*>(midCmd);
            if (nullptr != message)
            {
                std::cout << "This is a MessageType" << std::endl;
            }

            MoveScrewType* mvScrew = dynamic_cast<MoveScrewType*>(midCmd);
            if (nullptr != mvScrew)
            {
                std::cout << "This is a MoveScrewType" << std::endl;
            }

            MoveThroughToType* movethrough = dynamic_cast<MoveThroughToType*>(midCmd);
            if (nullptr != moveThrough)
            {
                std::cout << "This is a MoveThroughToType" << std::endl;
            }

            PoseAndSetType* poseSet = dynamic_cast<PoseAndSetType*>(midCmd);
            if (nullptr != poseSet)
            {
                std::cout << "This is a PoseAndSetType" << std::endl;
            }

            TransSpeedRelativeType* transSpeed = dynamic_cast<TransSpeedRelativeType*>(midCmd);
            if (nullptr != transSpeed)
            {
                std::cout << "This is a TransSpeedRelativeType" << std::endl;
            }

            TransAccelRelativeType* transAccel = dynamic_cast<TransAccelRelativeType*>(midCmd);
            if (nullptr != transAccel)
            {
                std::cout << "This is a TransAccelRelativeType" << std::endl;
            }

            OpenToolChangerType* opentool = dynamic_cast<OpenToolChangerType*>(midCmd);
            if (nullptr != opentool)
            {
                std::cout << "This is an OpenToolChangerType" << std::endl;
            }

            RunProgramType* runprog = dynamic_cast<RunProgramType*>(midCmd);
            if (nullptr != runprog)
            {
                std::cout << "This is a RunProgramType" << std::endl;
            }

            SetAngleUnitsType* setang = dynamic_cast<SetAngleUnitsType*>(midCmd);
            if (nullptr != setang)
            {
                std::cout << "This is a SetAngleUnitsType" << std::endl;
            }

            SetEndPoseToleranceType* endTol = dynamic_cast<SetEndPoseToleranceType*>(midCmd);
            if (nullptr != endTol)
            {
                std::cout << "This is a SetEndPoseToleranceType" << std::endl;
            }

            SetForceUnitsType* setforce = dynamic_cast<SetForceUnitsType*>(midCmd);
            if (nullptr != setforce)
            {
                std::cout << "This is a SetForceUnitsType" << std::endl;
            }

            SetIntermediatePoseToleranceType* setintpose = dynamic_cast<SetIntermediatePoseToleranceType*>(midCmd);
            if (nullptr != setintpose)
            {
                std::cout << "This is a SetIntermediatePoseToleranceType" << std::endl;
            }

            SetLengthUnitsType* setLenUnit = dynamic_cast<SetLengthUnitsType*>(midCmd);
            if (nullptr != setLenUnit)
            {
                std::cout << "This is a SetLengthUnitsType" << std::endl;
            }

            SetMotionCoordinationType* setmotcoor = dynamic_cast<SetMotionCoordinateType*>(midCmd);
            if (nullptr != setmotcoor)
            {
                std::cout << "This is a SetMotionCoordinateType" std::endl;
            }

            SetRobotParametersType* robotParam = dynamic_cast<SetRobotParametersType*>(midCmd);
            if (nullptr != robotParam)
            {
                std::cout << "This is a SetRobotParametersType" << std::endl;
            }

            SetRotAccelType* setrotacc = dynamic_cast<SetRotAccelType*>(midCmd);
            if (nullptr != setrotacc)
            {
                std::cout << "This is a SetRotAccelType" << std::endl;
            }

            SetRotSpeedType* setrotspeed = dynamic_cast<SetRotSpeedType*>(midCmd);
            if (nullptr != setrotspeed)
            {
                std::cout << "This is a SetRotSpeedType" << std::endl;
            }

            SetTorqueUnitsType* settorque = dynamic_cast<SetTorqueUnitsType*>(midCmd);
            if (nullptr != settorque)
            {
                std::cout << "This is a SetTorqueUnitsType" << std::endl;
            }

            SetTransAccelType* settransacc = dynamic_cast<SetTransAccelType*>(midCmd);
            if (nullptr != settransacc)
            {
                std::cout << "This is a SetTransAccelType" << std::endl;
            }

            SetTransSpeedType* settransspeed = dynamic_cast<SetTransSpeedType*>(midCmd);
            if (nullptr != settransspeed)
            {
                std::cout << "This is a SetTransSpeedType" << std::endl;
            }

            StopMotionType* stopmotion = dynamic_cast<StopMotionType*>(midCmd);
            if (nullptr != stopmotion)
            {
                std::cout << "This is a StopMotionType" << std::endl;
            }
            return;
        }
    }

} // namespace crcl
