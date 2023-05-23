// enable the clocks for desired peripherals (GPIOA, C and E)
void enable_clocks();

// initialise the discovery board I/O (just outputs: inputs are selected by default)
void initialise_board();

void TIM3_IRQHandler(void);

void enableUSART1();
