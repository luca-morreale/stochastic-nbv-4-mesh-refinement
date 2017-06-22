#include <opview/ReportWriter.hpp>

namespace opview {

    ReportWriter::ReportWriter(std::string reportFilename)
    {
        this->reportFilename = reportFilename;
        init();
    }

    ReportWriter::~ReportWriter()
    {

        long pos = out.tellp();
        out.seekp(pos-2);
        out << "\n]\n}\n";
        out.flush();
        out.close();
    }

    void ReportWriter::init()
    {
        this->out.open(reportFilename);

        out << "{\n" << "\"cams\": [ \n";
        out.flush();
    }

    void ReportWriter::append(DoubleList &pose, double score)
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
        out << "\"score:\": " << score << std::endl;

        out << "},\n";
        out.flush();
    }

    void ReportWriter::append(OrderedPose poses)
    {
        EigVector5 vec = poses.top().second;
        DoubleList pose;
        for (int i = 0; i < 5; i++) {
            pose.push_back(vec[i]);
        }
        append(pose, poses.top().first);
    }

    void ReportWriter::resetFile(std::string newfile)
    {
        out.close();
        std::remove(this->reportFilename.c_str());
        this->reportFilename = newfile;
        init();
    }


} // namespace opview
