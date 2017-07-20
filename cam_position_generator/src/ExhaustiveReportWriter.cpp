#include <opview/ExhaustiveReportWriter.hpp>

namespace opview {

    ExhaustiveReportWriter::ExhaustiveReportWriter(std::string reportFilename) : ReportWriter(reportFilename)
    { }

    ExhaustiveReportWriter::~ExhaustiveReportWriter()
    { }

    void ExhaustiveReportWriter::append(DoubleList &pose, double score, int round)
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

    void ExhaustiveReportWriter::append(OrderedPose poses, int round)
    {
        while (!poses.empty()) {
            EigVector5 vec = poses.top().second;
            DoubleList tmp = convert(vec);
            append(tmp, poses.top().first, round);
            poses.pop();
        }        
    }

    DoubleList ExhaustiveReportWriter::convert(EigVector5 &vec)
    {
        DoubleList pose;
        for (int i = 0; i < 5; i++) {
            pose.push_back(vec[i]);
        }
        return pose;
    }


} // namespace opview
