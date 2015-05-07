/**
 * @file modules/edgeflow_histogram/edgeflow.h
 *
 * Get images, calculates edge feature flow and sends them
 *
 * Works on Linux platforms
 */
#ifndef EDGE_FLOW_H
#define EDGE_FLOW_H

//Module Functions
extern void init_edgeflow(void);
extern void periodic_1Hz_edgeflow(void);
extern void run_edgeflow(void);
extern void start_edgeflow(void);
extern void stop_edgeflow(void);



#endif /*EDGE_FLOW_H*/
