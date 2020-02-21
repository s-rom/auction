#ifndef MATH_UTIL
#define MATH_UTIL

namespace Auction
{


    /**
     * Returns the task utility Uj in function of the elapsed time.
     * The function returns: 
     *      -> Uj = MAX_UTILITY  if elapsed_time < dead_line
     *      -> Uj = 0            otherwise
     * 
     * @param elapsed_time elapsed (millis) time from the beginning of the task
     * @param max_utility maximum value for utility
     * @param dead_line time before the task has any utility
     * @returns Uj(tj) where tj is the time elapsed since the task started (delta_time)
     */
    float hard_deadline_utility(float dead_line, float max_utility, float elapsed_time)
    {
        if (elapsed_time < dead_line)
            return max_utility;
        else
            return 0;
    }


    /**
     * Returns the task utility Uj in function of elapsed time.
     * The function returns:
     *      -> Uj = MAX_UTILITY             if elapsed_time < dead_line
     *      -> Uj = MAX_UTILITY * factor    otherwise
     *         where factor is
     *                 0.07 * dead_line
     *     ------------------------------------------
     *     (elapsed - dead_line) + (0.07 * dead_line)
     * 
     * @param elapsed_time elapsed (millis) time from the beginning of the task
     * @param max_utility maximum value for utility
     * @param dead_line time before the task has any utility
     * @returns Uj(tj) where tj is the time elapsed since the task started (delta_time) 
     */ 
    float soft_deadline_utility(float dead_line, float max_utility, float elapsed_time)
    {
        if (elapsed_time < dead_line)
        {
           return max_utility;
        }
        else 
        {
            float factor = (0.07 * dead_line) / 
                ((elapsed_time - dead_line) + (0.07 * dead_line));
            return max_utility * factor;
        }
             
    }

}
#endif