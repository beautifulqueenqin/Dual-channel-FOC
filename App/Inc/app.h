#ifndef APP_H_
#define APP_H_

/**
 * @brief Initialize the application and all board-support services.
 */
void App_Init(void);

/**
 * @brief Run one non-blocking application background iteration.
 */
void App_Loop(void);

#endif /* APP_H_ */
