#pragma once

template <typename IteratorType_>
int bruteForceSearch(std::vector<typename IteratorType_::value_type*>& answers,
                     IteratorType_ begin,
                     IteratorType_ end,
                     const typename IteratorType_::value_type& query,
                     const typename IteratorType_::value_type::Scalar norm) {
  using Scalar = typename IteratorType_::value_type::Scalar;
  const Scalar  squared_norm = norm*norm;
  int matches=0;
  for (auto it=begin; it!=end; ++it) {
    auto& p=*it;
    if ((p-query).squaredNorm()<squared_norm) {
      answers.push_back(&p);
      ++matches;
    }
  }
  return matches;
}

template <typename IteratorType_>
void bruteForceSearchCustom(typename IteratorType_::value_type* answers,
                     IteratorType_ begin,
                     IteratorType_ end,
                     const typename IteratorType_::value_type& query) {
  using Scalar = typename IteratorType_::value_type::Scalar;
  Scalar best_distance = 999.0f;
  
  for (auto it=begin; it!=end; ++it) {
    auto& p=*it;

    float current_distance = (p-query).squaredNorm();

    if ( current_distance < best_distance) {
      answers = &p;
      best_distance = current_distance;

    }
  }
}