#include "math.h"
#include <Arduino.h>

int abs( int n ) {
	return ( n < 0 ) ? -n : n;
}
int min( const int &a, const int &b ) {
	return ( a < b ) ? a : b;
}
int max( const int &a, const int &b ) {
	return ( a < b ) ? b : a;
}
