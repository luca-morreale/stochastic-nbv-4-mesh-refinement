#pragma once
#ifndef OPENGM_MULTIBRUTEFORCE_HXX
#define OPENGM_MULTIBRUTEFORCE_HXX

#include "inference.hxx"
#include "movemaker.hxx"
#include "opengm/inference/visitors/visitors.hxx"
#include "opengm/inference/bruteforce.hxx"

#include <queue>

namespace opengm {

template<class GM> class Movemaker;

/// Brute force inference algorithm 
///
/// \ingroup inference
template<class GM, class ACC>
class MultiBruteforce : public Bruteforce<GM, ACC>
{
private:
    typedef std::vector<typename Bruteforce<GM, ACC>::LabelType> State;
    typedef std::pair<typename Bruteforce<GM, ACC>::ValueType, State> ValueStatePair;

    struct ValueStatePairCompare{
      bool operator()(ValueStatePair const& lhs, ValueStatePair const& rhs){
        return lhs.first < rhs.first;
      }
    };

    template<typename T>                            // NOTE custom addition
    class fixed_priority_queue : public std::priority_queue<T> 
    {
      public:
        fixed_priority_queue(unsigned int size) : fixed_size(size) {}
        void push(const T& x)
        {
          // If we've reached capacity, find the FIRST smallest object and replace
          // it if 'x' is larger
          if(this->size() == fixed_size)
          {
            // 'c' is the container used by priority_queue and is a protected member.
            auto beg = this->c.begin(); auto end = this->c.end();
            auto min = std::min_element(beg, end, ValueStatePairCompare());
            if(x > *min)
            {
                *min = x;
                // Re-make the heap, since we may have just invalidated it.
                std::make_heap(beg, end, ValueStatePairCompare());
            }
          } else { // Otherwise just push the new item.
            priority_queue<T>::push(x);
          }
        }

        fixed_priority_queue operator=(fixed_priority_queue&& other)
        {
            this->c = other.c;
        }
        fixed_priority_queue operator=(fixed_priority_queue& other)
        {
            this->c = other.c;
        }

      private:
        fixed_priority_queue() {} // Construct with size only.
        const unsigned int fixed_size;
        // Prevent heap allocation
        void * operator new   (size_t);
        void * operator new[] (size_t);
        void   operator delete   (void *);
        void   operator delete[] (void*);
    };

    size_t limit;
    fixed_priority_queue<ValueStatePair> statesList;

public:
    MultiBruteforce(const typename Bruteforce<GM, ACC>::GraphicalModelType&, size_t limit=10);
    MultiBruteforce(const typename Bruteforce<GM, ACC>::GraphicalModelType&, const typename Bruteforce<GM, ACC>::Parameter&, size_t limit=10);
    InferenceTermination infer()                     { typename Bruteforce<GM, ACC>::EmptyVisitorType visitor; return infer(visitor);}
    template<class VISITOR> InferenceTermination infer(VISITOR &);
    InferenceTermination argAll(std::priority_queue<std::pair<typename Bruteforce<GM, ACC>::ValueType, std::vector<typename Bruteforce<GM, ACC>::LabelType>>> &states) const;
    
    void setLimitQueue(size_t limit) { this->limit = limit; this->statesList = fixed_priority_queue<ValueStatePair>(limit); };
    size_t limitQueue() { return limit; }


};
template<class GM, class AKK>
MultiBruteforce<GM, AKK>::MultiBruteforce
(
    const typename Bruteforce<GM, AKK>::GraphicalModelType& gm, 
    size_t limit
)
:  Bruteforce<GM, AKK>(gm), limit(limit), statesList(limit)
{ }

template<class GM, class AKK>
MultiBruteforce<GM, AKK>::MultiBruteforce
(
   const typename Bruteforce<GM, AKK>::GraphicalModelType& gm,
   const typename Bruteforce<GM, AKK>::Parameter& param,
   size_t limit
)
:  Bruteforce<GM, AKK>(gm, param), limit(limit), statesList(limit)
{ }

template<class GM, class AKK>
template<class VISITOR>
InferenceTermination
MultiBruteforce<GM, AKK>::infer
(
   VISITOR & visitor
)
{
   std::vector<typename Bruteforce<GM, AKK>::LabelType> states(graphicalModel().numberOfVariables());
   std::vector<typename Bruteforce<GM, AKK>::IndexType> vi(graphicalModel().numberOfVariables());
   for(size_t j=0; j<graphicalModel().numberOfVariables(); ++j) {
       vi[j] = j;
   }
   typename Bruteforce<GM, AKK>::ValueType energy_ = this->energy();

   Bruteforce<GM, AKK>::AccumulationType::neutral(energy_); 
   bool exitInf = false;
   visitor.begin(*this);
   while(exitInf == false) {
      typename Bruteforce<GM, AKK>::ValueType energy = moveMaker().move(vi.begin(), vi.end(), states.begin());
      
      if(Bruteforce<GM, AKK>::AccumulationType::bop(energy , energy_)) {
         setStates(states);
         statesList.push(std::make_pair(energy, states));       // NOTE custom addition
      } 
      
      Bruteforce<GM, AKK>::AccumulationType::op(energy, energy_); 
      setEnergy(energy_);

      if( visitor(*this) != visitors::VisitorReturnFlag::ContinueInf ){
         exitInf = true;
      }
      bool overflow = true;
      for(size_t j=0; j<graphicalModel().numberOfVariables(); ++j) {
         if( size_t(states[j]+1) < size_t(graphicalModel().numberOfLabels(j))) {
            ++states[j];
            for(size_t k=0; k<j; ++k) {
               states[k] = 0;
            }
            overflow = false;
            break;
         }
      }
      if(overflow) {
         break;
      }
   }
   visitor.end(*this);
   return NORMAL;
}


template<class GM, class AKK>
inline InferenceTermination
MultiBruteforce<GM, AKK>::argAll
(
   std::priority_queue<ValueStatePair> &states
) const
{
    fixed_priority_queue<ValueStatePair> copy(this->limit);
    
    while(!statesList.empty()) {
        states.push(statesList.top());
        copy.push(statesList.top());
        statesList.pop();
    }

    statesList = copy;
}


} // namespace opengm

#endif // #ifndef OPENGM_BRUTEFORCE_HXX
