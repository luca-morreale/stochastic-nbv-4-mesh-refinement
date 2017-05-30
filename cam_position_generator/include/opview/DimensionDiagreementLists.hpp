#ifndef CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H
#define CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H


namespace opview {

    class DimensionDiagreementLists : public std::runtime_error {
    public:
        DimensionDiagreementLists(std::string msg) : std::runtime_error(msg) { }
        DimensionDiagreementLists(size_t lenA, size_t lenB) 
            : std::runtime_error("Dimension of lists do not match, " + std::to_string(lenA) + "!=" + std::to_string(lenB) +".\n") { }
        DimensionDiagreementLists() : std::runtime_error("Dimension of lists do not match.\n") { }

        ~DimensionDiagreementLists() { }

    };


} // namespace opview

#endif // CAM_POSITION_DIMENSION_DISAGREEMENT_LISTS_H
