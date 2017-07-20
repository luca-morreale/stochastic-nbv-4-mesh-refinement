#ifndef CAM_POSITION_COMPLETE_REPORT_WRITER_H
#define CAM_POSITION_COMPLETE_REPORT_WRITER_H

#include <opview/alias_definition.h>
#include <opview/ReportWriter.hpp>

namespace opview {
    
    class ExhaustiveReportWriter : public ReportWriter {
    public:
        ExhaustiveReportWriter(std::string reportFilename);
        ~ExhaustiveReportWriter();

        virtual void append(OrderedPose poses, int round);
        virtual void append(DoubleList &pose, double score, int round);

    protected:
        virtual DoubleList convert(EigVector5 &vec);

    };

    typedef ExhaustiveReportWriter* ExhaustiveReportWriterPtr;


} // namespace opview

#endif // CAM_POSITION_COMPLETE_REPORT_WRITER_H
