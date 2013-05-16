/**
 * \file ft_flag_t.h
 * \brief Enumerates possible return values for functions in the Fastrak class.
 *
 * \author Andrew Price
 */
#ifndef FT_FLAG_T_H
#define FT_FLAG_T_H

/**
 * \enum ft_flag_t
 * \brief Enumerates possible return values for functions in the Fastrak class.
 */
typedef enum
{
    SUCCESS = 0,	///< The command returned successfully
    SENSOR_OOB,     ///< You requested data from a sensor which doesn't exist
    FASTRAK_STALE,  ///< The Fastrak values were not able to update for some reason
    CHAN_OPEN_FAIL, ///< A channel failed to open
} ft_flag_t;

// Consider converting to enum class for type-safe operation

#endif //FT_FLAG_T_H
