#!/bin/bash
# View recorded attack connections

echo "=========================================="
echo "Attack Connection Logs"
echo "=========================================="
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$PROJECT_ROOT/logs/monitoring"

if [ ! -d "$LOG_DIR" ]; then
    echo "⚠️  Monitoring log directory does not exist: $LOG_DIR"
    echo "   No attack connections have been monitored yet."
    exit 0
fi

# Find all log files
LOG_FILES=$(find "$LOG_DIR" -name "attack_connections_*.log" -type f | sort -r)

if [ -z "$LOG_FILES" ]; then
    echo "⚠️  No attack connection logs found"
    echo ""
    echo "To start monitoring, run:"
    echo "  ./scripts/monitoring/monitor_attack_connections.sh"
    exit 0
fi

echo "Found $(echo "$LOG_FILES" | wc -l) log file(s)"
echo ""

# Show latest log
LATEST_LOG=$(echo "$LOG_FILES" | head -1)
echo "📋 Latest log: $(basename "$LATEST_LOG")"
echo "=========================================="
echo ""

# Show alerts and warnings
ALERTS=$(grep -i "ALERT\|🚨" "$LATEST_LOG" 2>/dev/null)
if [ -n "$ALERTS" ]; then
    echo "🚨 ALERTS DETECTED:"
    echo "$ALERTS"
    echo ""
fi

# Show all entries
echo "All entries:"
echo "----------------------------------------"
tail -50 "$LATEST_LOG"
echo ""

# Show summary
echo "=========================================="
echo "Summary"
echo "=========================================="
TOTAL_ALERTS=$(grep -c "🚨 ALERT" "$LATEST_LOG" 2>/dev/null || echo "0")
TOTAL_WARNINGS=$(grep -c "⚠️" "$LATEST_LOG" 2>/dev/null || echo "0")

echo "Total alerts: $TOTAL_ALERTS"
echo "Total warnings: $TOTAL_WARNINGS"
echo ""
echo "View full log:"
echo "  cat $LATEST_LOG"
echo ""

