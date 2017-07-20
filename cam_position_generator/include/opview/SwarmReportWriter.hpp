#ifndef CAM_POSITION_SWARM_REPORT_WRITER_H
#define CAM_POSITION_SWARM_REPORT_WRITER_H

#include <opview/Particle.hpp>
#include <opview/ExhaustiveReportWriter.hpp>

namespace opview {
    
    class SwarmReportWriter : public ExhaustiveReportWriter {
    public:
        SwarmReportWriter(std::string reportFilename);
        ~SwarmReportWriter();

        virtual void append(ParticleList &poses, int round);
        using ExhaustiveReportWriter::append;
    
    private:
        typedef ExhaustiveReportWriter super;

    };

    typedef SwarmReportWriter* SwarmReportWriterPtr;


} // namespace opview

#endif // CAM_POSITION_SWARM_REPORT_WRITER_H
