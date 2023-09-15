#!/bin/bash -eu

COVERITY_SUMMARY_OUT=/main_ws/cov-analyze-result.txt
COVERITY_SCAN_OUT=/main_ws/cov-scan-output.txt
COVERITY_REPORT_OUT=/main_ws/coverity-output

cp /main_ws/packaging/coverity.yaml /main_ws/coverity.yaml

export PATH=$PATH:/cov/bin/
cov-configure --gcc
coverity scan --exclude-language java |tee ${COVERITY_SCAN_OUT}
coverity list

# find important information from coverity scan to be shown on github action step summary
# link for coverity
grep 'Results are available at' ${COVERITY_SCAN_OUT} >> ${COVERITY_SUMMARY_OUT}
echo 'send a slack message to tampere-drones if you have access issues' >> ${COVERITY_SUMMARY_OUT}

echo "Analysis summary:">> ${COVERITY_SUMMARY_OUT}
# '```' marks the code block for output
echo '```' >> ${COVERITY_SUMMARY_OUT}
# check analysis summary output and save everything beginning from "analysis summary report" to a file
cov-analyze --dir idir --strip-path /main_ws/src/ |sed -n -E -e '/Analysis summary report:/,$ p'>>${COVERITY_SUMMARY_OUT}
# '```' ends the code block for output
echo '```' >> ${COVERITY_SUMMARY_OUT}

echo "File findings:">> ${COVERITY_SUMMARY_OUT}
echo '------' >> ${COVERITY_SUMMARY_OUT}

# save coverity html output
cov-format-errors --dir idir --html-output ${COVERITY_REPORT_OUT}
echo 'for more details please check attached html report from "Artifacts" -sections above' >> ${COVERITY_SUMMARY_OUT}
