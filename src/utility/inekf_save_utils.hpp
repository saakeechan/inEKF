#ifndef SRC_INEKF_SAVE_UTILS_HPP
#define SRC_INEKF_SAVE_UTILS_HPP

#include <vector>
#include <string>

// Forward declaration
class InEKFilter;

namespace InEKF_SaveUtils {

    /**
     * @brief Save one timestep of estimator data to SAVE_MATRIX
     * @param filter Reference to the InEKFilter instance
     * @param save_index Index in SAVE_MATRIX to write to
     */
    void save_onestep_impl(InEKFilter& filter, int save_index);

    /**
     * @brief Write all saved data to CSV file with timestamp-based filename
     * @param filter Reference to the InEKFilter instance
     */
    void write_saved_data_impl(InEKFilter& filter);

    /**
     * @brief Print contact detection accuracy statistics to stdout
     * @param filter Reference to the InEKFilter instance
     */
    void print_contact_accuracy_impl(const InEKFilter& filter);

} // namespace InEKF_SaveUtils

#endif // SRC_INEKF_SAVE_UTILS_HPP
