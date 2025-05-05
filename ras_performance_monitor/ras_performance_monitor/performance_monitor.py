import time
from typing import Optional, Dict, Any
from ras_logging import RasLogger

class PerformanceMonitor:
    def __init__(self):
        self._timers: Dict[str, float] = {}
        self._logger = RasLogger()
        
    def start_timer(self, name: str) -> None:
        """Start a timer with the given name."""
        self._timers[name] = time.time()
        
    def stop_timer(self, name: str) -> float:
        """Stop a timer and return the elapsed time in seconds."""
        if name not in self._timers:
            raise ValueError(f"Timer '{name}' was never started")
            
        elapsed = time.time() - self._timers[name]
        del self._timers[name]
        return elapsed
        
    def measure(self, name: str, func: callable, *args, **kwargs) -> Any:
        """Measure the execution time of a function."""
        self.start_timer(name)
        result = func(*args, **kwargs)
        elapsed = self.stop_timer(name)
        self._logger.log_info(f"[Performance] {name} took {elapsed:.4f} seconds")
        return result
        
    def measure_block(self, name: str) -> 'PerformanceMonitor.Context':
        """Measure the execution time of a code block."""
        return self.Context(self, name)
        
    class Context:
        def __init__(self, monitor: 'PerformanceMonitor', name: str):
            self._monitor = monitor
            self._name = name
            
        def __enter__(self):
            self._monitor.start_timer(self._name)
            
        def __exit__(self, exc_type, exc_val, exc_tb):
            elapsed = self._monitor.stop_timer(self._name)
            self._monitor._logger.log_info(f"[Performance] {self._name} took {elapsed:.4f} seconds")

# Global instance for easy access
global_performance_monitor = PerformanceMonitor()
