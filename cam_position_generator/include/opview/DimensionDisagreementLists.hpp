#ifndef CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H
#define CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H

#include <stdexcept>

namespace opview {

    class DimensionDisagreementLists : public std::runtime_error {
    public:
        DimensionDisagreementLists(std::string msg) : std::runtime_error(msg) { }
        DimensionDisagreementLists(size_t lenA, size_t lenB) 
            : std::runtime_error("Dimension of lists do not match, " + std::to_string(lenA) + "!=" + std::to_string(lenB) +".\n") { }
        DimensionDisagreementLists() : std::runtime_error("Dimension of lists do not match.\n") { }

        ~DimensionDisagreementLists() { }

    };


} // namespace opview

#endif // CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H
