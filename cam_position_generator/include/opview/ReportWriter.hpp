#ifndef CAM_POSITION_REPORT_WRITER_H
#define CAM_POSITION_REPORT_WRITER_H

#include <fstream>
#include <opview/alias_definition.h>
#include <opview/type_definition.h>

namespace opview {
    
    // in case of single point it just estimate the best point using a weighted function that consider all visible points
    
    class ReportWriter {
    public:
        ReportWriter(std::string reportFilename);
        ~ReportWriter();

        virtual void append(DoubleList &pose, double score);
        virtual void append(OrderedPose poses);

        void resetFile(std::string newfile);
    
    
    private:
        std::string reportFilename;
        std::ofstream out;

        void init();
    };

    typedef ReportWriter* ReportWriterPtr;


} // namespace opview

#endif // CAM_POSITION_REPORT_WRITER_H
