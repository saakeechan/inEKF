#ifndef SRC_INEKF_SAVE_UTILS_HPP
#define SRC_INEKF_SAVE_UTILS_HPP

#include <vector>
#include <string>

// Forward declaration
class INEKFilter_manager;

namespace InEKF_SaveUtils {

    // Reset logging history and contact-accuracy counters.
    void initialize_logging(INEKFilter_manager& filter);

    /**
     * @brief Save one timestep of estimator data to SAVE_MATRIX
     * @param filter Reference to the INEKFilter_manager instance
     * @param save_index Index in SAVE_MATRIX; 0 is initialization, CSV source row is start_index + save_index
     */
    void save_onestep_impl(INEKFilter_manager& filter, int save_index);

    /**
     * @brief Write all saved data to CSV file with timestamp-based filename
     * @param filter Reference to the INEKFilter_manager instance
     */
    void write_saved_data_impl(INEKFilter_manager& filter);

    /**
     * @brief Print contact detection accuracy statistics to stdout
     * @param filter Reference to the INEKFilter_manager instance
     */
    void print_contact_accuracy_impl(const INEKFilter_manager& filter);

} // namespace InEKF_SaveUtils

#endif // SRC_INEKF_SAVE_UTILS_HPP
