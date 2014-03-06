/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/Timestamp.hpp>
#include <uavcan/FigureOfMerit.hpp>
#include <uavcan/mavlink/Message.hpp>
#include <uavcan/protocol/ComputeAggregateTypeSignature.hpp>
#include <uavcan/protocol/GetProtocolStatistics.hpp>
#include <uavcan/protocol/Panic.hpp>
#include <uavcan/protocol/RestartNode.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>
#include <uavcan/protocol/DataTypeKind.hpp>
#include <uavcan/protocol/GlobalDiscoveryRequest.hpp>
#include <uavcan/protocol/GetDataTypeInfo.hpp>
#include <uavcan/protocol/Version.hpp>
#include <uavcan/protocol/NodeStatus.hpp>
#include <uavcan/protocol/GetNodeInfo.hpp>
#include <uavcan/protocol/debug/LogMessage.hpp>
#include <uavcan/protocol/debug/StartHilSimulation.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include <uavcan/protocol/file/Path.hpp>
#include <uavcan/protocol/file/Read.hpp>
#include <uavcan/protocol/file/Delete.hpp>
#include <uavcan/protocol/file/Errno.hpp>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/protocol/file/List.hpp>
#include <uavcan/protocol/file/BeginTransfer.hpp>
#include <uavcan/protocol/file/Crc.hpp>
#include <uavcan/protocol/file/GetInfo.hpp>
