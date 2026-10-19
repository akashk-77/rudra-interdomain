/**
 * @file fsm_common.h
 * @brief Types shared by the rover FSM and the mechanism FSM.
 *
 * Every hardware/software service that can take time (vision, path planning,
 * driving, ...) is exposed to the FSMs as a function that returns a
 * hal_status_t. The FSMs call it again on every update() until it stops
 * returning HAL_BUSY. This keeps the FSMs NON-BLOCKING: update() must
 * return quickly so that both FSMs can run in the same super-loop / RTOS tick.
 */
#ifndef FSM_COMMON_H
#define FSM_COMMON_H

typedef enum {
    HAL_BUSY = 0,   /**< Still working, call again next update().          */
    HAL_OK   = 1,   /**< Finished successfully.                             */
    HAL_FAIL = -1   /**< Finished with failure (FSM decides retry / error). */
} hal_status_t;

#endif /* FSM_COMMON_H */
