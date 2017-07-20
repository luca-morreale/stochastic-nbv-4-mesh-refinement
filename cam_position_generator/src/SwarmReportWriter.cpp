#include <opview/SwarmReportWriter.hpp>

namespace opview {

    SwarmReportWriter::SwarmReportWriter(std::string reportFilename) : ExhaustiveReportWriter(reportFilename)
    { }

    SwarmReportWriter::~SwarmReportWriter()
    { }

    void SwarmReportWriter::append(ParticleList &poses, int round)
    {
        for (int i = 0; i < poses.size(); i++) {
            DoubleList tmp = convert(poses[i]->position);
            super::append(tmp, poses[i]->value, round);
        }
    }

} // namespace opview
