#include <opview/CompleteReportWriter.hpp>

namespace opview {

    CompleteReportWriter::CompleteReportWriter(std::string reportFilename) : ReportWriter(reportFilename)
    { }

    CompleteReportWriter::~CompleteReportWriter()
    { }

    void CompleteReportWriter::append(DoubleList &pose, double score, int round)
    {
        out << "{\n" << "\"vals\": [";

        for (int i = 0; i < 3; i++) {
            out << pose[i] << ", ";
        }

        long pos = out.tellp();
        out.seekp(pos-2);
        out << "], \n";

        out << "\"rots\": [";

        for (int i = 3; i < 5; i++) {
            out << pose[i] << ", ";
        }

        pos = out.tellp();
        out.seekp(pos-2);
        out << "], \n";
        out << "\"score\": " << score << ", " << std::endl;
        out << "\"round\": " << round << std::endl;

        out << "},\n";
        out.flush();
    }

    void CompleteReportWriter::append(OrderedPose poses, int round)
    {
        while (!poses.empty()) {
            EigVector5 vec = poses.top().second;
            auto tmp = convert(vec);
            append(tmp, poses.top().first, round);
            poses.pop();
        }        
    }

    DoubleList CompleteReportWriter::convert(EigVector5 &vec)
    {
        DoubleList pose;
        for (int i = 0; i < 5; i++) {
            pose.push_back(vec[i]);
        }
        return pose;
    }


} // namespace opview
