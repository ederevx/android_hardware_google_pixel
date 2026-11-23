/*
 * Copyright (C) 2026 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "AdpfDiag.h"

#include <android-base/stringprintf.h>

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <new>
#include <pthread.h>

using ::android::base::StringPrintf;

namespace aidl::google::hardware::power::impl::pixel {

namespace {

constexpr size_t kWindowIdLength = 32;
constexpr size_t kWriteChunk = 64 * 1024;

bool readAll(const char *path, std::string *out) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return false;
    }
    out->clear();
    char buffer[256];
    while (true) {
        ssize_t count = read(fd, buffer, sizeof(buffer));
        if (count > 0) {
            out->append(buffer, count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        close(fd);
        return count == 0;
    }
}

std::string trim(const std::string &text) {
    size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

// Field 22 of /proc/<pid>/stat is the process start time in clock ticks; the
// fields between the process name (which may contain spaces and parentheses) and
// start time are all numeric, so parsing resumes after the last ')'.
uint64_t parseStartTicks(const std::string &stat) {
    size_t close = stat.rfind(')');
    if (close == std::string::npos) {
        return 0;
    }
    const char *cursor = stat.c_str() + close + 1;
    // starttime is field 22; the fields after ')' begin at field 3.
    for (int field = 3; field < 22; ++field) {
        while (*cursor == ' ') {
            ++cursor;
        }
        while (*cursor != '\0' && *cursor != ' ') {
            ++cursor;
        }
    }
    while (*cursor == ' ') {
        ++cursor;
    }
    return strtoull(cursor, nullptr, 10);
}

void appendJsonString(std::string *out, const std::string &value) {
    out->push_back('"');
    for (unsigned char byte : value) {
        if (byte == '"' || byte == '\\') {
            out->push_back('\\');
            out->push_back(static_cast<char>(byte));
        } else if (byte >= 0x20 && byte <= 0x7e) {
            out->push_back(static_cast<char>(byte));
        } else {
            out->append(StringPrintf("\\u%04x", byte));
        }
    }
    out->push_back('"');
}

void appendJsonNumber(std::string *out, double value) {
    if (!std::isfinite(value)) {
        out->append("null");
        return;
    }
    out->append(StringPrintf("%.17g", value));
}

void appendJsonNullableNumber(std::string *out, bool present, double value) {
    if (!present) {
        out->append("null");
        return;
    }
    appendJsonNumber(out, value);
}

void appendJsonBits(std::string *out, bool present, uint64_t bits) {
    if (!present) {
        out->append("null");
        return;
    }
    out->append("\"");
    out->append(StringPrintf("0x%016" PRIx64, bits));
    out->append("\"");
}

uint64_t doubleBits(double value) {
    uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

const std::string &lookupName(const std::vector<std::string> &names, uint16_t id) {
    static const std::string kEmpty;
    if (id < names.size()) {
        return names[id];
    }
    return kEmpty;
}

double doubleFromBits(uint64_t bits) {
    double value = 0;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool isLowerHex(const std::string &text) {
    return std::all_of(text.begin(), text.end(), [](char c) {
        return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
}

void splitWindowId(const std::string &windowId, uint64_t *high, uint64_t *low) {
    *high = strtoull(windowId.substr(0, 16).c_str(), nullptr, 16);
    *low = strtoull(windowId.substr(16, 16).c_str(), nullptr, 16);
}

std::string joinWindowId(uint64_t high, uint64_t low) {
    return StringPrintf("%016" PRIx64 "%016" PRIx64, high, low);
}

// A dumpsys reader that disappears during a long read must not kill the HAL
// through SIGPIPE, so the signal is blocked for this thread around the writes.
class SigpipeGuard {
  public:
    SigpipeGuard() {
        sigemptyset(&mSet);
        sigaddset(&mSet, SIGPIPE);
        mBlocked = pthread_sigmask(SIG_BLOCK, &mSet, &mOld) == 0;
    }
    ~SigpipeGuard() {
        if (!mBlocked) {
            return;
        }
        timespec zero = {0, 0};
        while (sigtimedwait(&mSet, nullptr, &zero) == SIGPIPE) {
        }
        pthread_sigmask(SIG_SETMASK, &mOld, nullptr);
    }

  private:
    sigset_t mSet;
    sigset_t mOld;
    bool mBlocked{false};
};

}  // namespace

const char *toString(DiagState state) {
    switch (state) {
        case DiagState::kActive:
            return "ACTIVE";
        case DiagState::kPaused:
            return "PAUSED";
        case DiagState::kClosed:
            return "CLOSED";
        case DiagState::kDestroyed:
            return "DESTROYED";
        default:
            return "NONE";
    }
}

const char *toString(DiagEvent event) {
    switch (event) {
        case DiagEvent::kCreate:
            return "CREATE";
        case DiagEvent::kPause:
            return "PAUSE";
        case DiagEvent::kResume:
            return "RESUME";
        case DiagEvent::kClose:
            return "CLOSE";
        case DiagEvent::kDestroy:
            return "DESTROY";
        case DiagEvent::kProfileSwap:
            return "PROFILE_SWAP";
        case DiagEvent::kRejected:
            return "REJECTED";
        case DiagEvent::kSfCreateWithLive:
            return "SF_CREATE_WITH_LIVE";
        case DiagEvent::kSfSuperseded:
            return "SF_SUPERSEDED";
        case DiagEvent::kBegin:
            return "BEGIN";
        case DiagEvent::kEnd:
            return "END";
        default:
            return "NONE";
    }
}

const char *toString(DiagOrigin origin) {
    switch (origin) {
        case DiagOrigin::kClient:
            return "client";
        case DiagOrigin::kDestructor:
            return "destructor";
        case DiagOrigin::kProfileCallback:
            return "profile_callback";
        default:
            return "none";
    }
}

const char *toString(DiagReason reason) {
    switch (reason) {
        case DiagReason::kClosed:
            return "closed";
        case DiagReason::kNoTarget:
            return "no_target";
        case DiagReason::kEmptyBatch:
            return "empty_batch";
        case DiagReason::kPaused:
            return "paused";
        case DiagReason::kPidOff:
            return "pid_off";
        case DiagReason::kHboostOff:
            return "hboost_off";
        case DiagReason::kNotActive:
            return "not_active";
        case DiagReason::kAlreadyActive:
            return "already_active";
        default:
            return "none";
    }
}

AdpfDiag::AdpfDiag(const Sources &sources)
    : mBootId(trim([&sources] {
          std::string value;
          if (!readAll(sources.bootIdPath, &value)) {
              return std::string();
          }
          return value;
      }())),
      mProcessStartTicks([&sources] {
          std::string value;
          if (!readAll(sources.statPath, &value)) {
              return static_cast<uint64_t>(0);
          }
          return parseStartTicks(value);
      }()),
      mPid(getpid()),
      mHalInstanceId(StringPrintf("%s/%" PRId32 "/%" PRIu64, mBootId.c_str(), mPid,
                                  mProcessStartTicks)),
      mRing(sources.allocateRing
                    ? std::unique_ptr<DiagRecord[]>(new (std::nothrow) DiagRecord[kRingCapacity])
                    : nullptr) {}

uint16_t AdpfDiag::internName(const std::string &name) {
    std::scoped_lock lock{mMutex};
    for (size_t index = 0; index < mNames.size(); ++index) {
        if (mNames[index] == name) {
            return static_cast<uint16_t>(index);
        }
    }
    if (mNames.size() >= kMaxNames) {
        ++mNameOverflow;
        return kIdOverflow;
    }
    mNames.push_back(name);
    return static_cast<uint16_t>(mNames.size() - 1);
}

bool AdpfDiag::appendLocked(DiagRecord &record, uint64_t *seqOut) {
    if (!mRing) {
        ++mFailedInserts;
        return false;
    }
    uint64_t seq = mNextSeq++;
    if (seq > kRingCapacity) {
        ++mOverwritten;
    }
    record.dropCount = mOverwritten;
    timespec now = {};
    clock_gettime(CLOCK_BOOTTIME, &now);
    record.boottimeNs = static_cast<int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
    mRing[(seq - 1) % kRingCapacity] = record;
    *seqOut = seq;
    return true;
}

uint64_t AdpfDiag::oldestRetainedLocked() const {
    uint64_t last = mNextSeq - 1;
    if (last <= kRingCapacity) {
        return 1;
    }
    return last - kRingCapacity + 1;
}

void AdpfDiag::appendDecision(const AdpfDiagDecisionContext &context, uint32_t itemIndex,
                              int64_t reportedDurationNs, int64_t targetDurationNs,
                              double factorUsed, double thresholdUs, bool missed) {
    DiagRecord record;
    record.kind = static_cast<uint8_t>(DiagKind::kDecision);
    record.state = static_cast<uint8_t>(context.state);
    record.instanceId = context.instanceId;
    record.generation = context.generation;
    record.reportSeq = context.reportSeq;
    record.itemIndex = itemIndex;
    record.procTagId = context.procTagId;
    record.sessTagId = context.sessTagId;
    record.profileId = context.profileId;
    record.a = targetDurationNs;
    record.b = reportedDurationNs;
    record.factorBits = doubleBits(factorUsed);
    record.hasFactor = 1;
    record.thresholdUs = thresholdUs;
    record.detail = missed ? 1 : 0;
    uint64_t seq = 0;
    std::scoped_lock lock{mMutex};
    appendLocked(record, &seq);
}

void AdpfDiag::appendUnclassified(int64_t instanceId, uint32_t generation, uint64_t reportSeq,
                                  uint16_t procTagId, uint16_t sessTagId, uint32_t batchSize,
                                  DiagReason reason, DiagState state) {
    DiagRecord record;
    record.kind = static_cast<uint8_t>(DiagKind::kReportUnclassified);
    record.state = static_cast<uint8_t>(state);
    record.instanceId = instanceId;
    record.generation = generation;
    record.reportSeq = reportSeq;
    record.procTagId = procTagId;
    record.sessTagId = sessTagId;
    record.a = batchSize;
    record.reason = static_cast<uint8_t>(reason);
    uint64_t seq = 0;
    std::scoped_lock lock{mMutex};
    appendLocked(record, &seq);
}

void AdpfDiag::appendLifecycle(const AdpfDiagLifecycle &event) {
    DiagRecord record;
    record.kind = static_cast<uint8_t>(DiagKind::kLifecycle);
    record.state = static_cast<uint8_t>(event.newState);
    record.detail = static_cast<uint8_t>(event.previousState);
    record.event = static_cast<uint8_t>(event.event);
    record.origin = static_cast<uint8_t>(event.origin);
    record.reason = static_cast<uint8_t>(event.reason);
    record.instanceId = event.instanceId;
    record.generation = event.generation;
    record.reportSeq = event.lastReportSeq;
    record.procTagId = event.procTagId;
    record.sessTagId = event.sessTagId;
    record.profileId = event.profileId;
    record.profileId2 = event.currentProfileId;
    record.hasFactor = event.hasFactor ? 1 : 0;
    record.factorBits = doubleBits(event.factor);
    record.a = static_cast<int64_t>(doubleBits(event.newProfileFactor));
    record.b = event.newHasFactor ? 1 : 0;

    std::scoped_lock lock{mMutex};
    uint64_t seq = 0;
    appendLocked(record, &seq);
    updateSfLocked(event, seq);
}

void AdpfDiag::updateSfLocked(const AdpfDiagLifecycle &event, uint64_t seq) {
    if (!event.isSurfaceFlinger) {
        return;
    }
    switch (event.event) {
        case DiagEvent::kCreate: {
            for (const auto &[instanceId, entry] : mLiveSf) {
                DiagRecord record;
                record.kind = static_cast<uint8_t>(DiagKind::kSfCompetition);
                record.event = static_cast<uint8_t>(DiagEvent::kSfCreateWithLive);
                record.state = static_cast<uint8_t>(event.newState);
                record.detail = static_cast<uint8_t>(entry.state);
                record.instanceId = event.instanceId;
                record.generation = event.generation;
                record.sessTagId = event.sessTagId;
                record.a = instanceId;
                record.b = entry.generation;
                uint64_t ignored = 0;
                appendLocked(record, &ignored);
            }
            SessionEntry entry;
            entry.instanceId = event.instanceId;
            entry.generation = event.generation;
            entry.state = event.newState;
            entry.createSeq = seq;
            mLiveSf[event.instanceId] = entry;
            break;
        }
        case DiagEvent::kPause:
        case DiagEvent::kResume: {
            auto it = mLiveSf.find(event.instanceId);
            if (it != mLiveSf.end()) {
                it->second.state = event.newState;
                it->second.generation = event.generation;
            }
            break;
        }
        case DiagEvent::kClose:
        case DiagEvent::kDestroy: {
            // A supersede needs a live session created after this one: an older
            // session closing is not evidence that anything took over.
            auto closing = mLiveSf.find(event.instanceId);
            const uint64_t closingCreate = closing != mLiveSf.end() ? closing->second.createSeq : 0;
            const SessionEntry *successor = nullptr;
            for (const auto &[instanceId, entry] : mLiveSf) {
                if (instanceId == event.instanceId || entry.createSeq <= closingCreate) {
                    continue;
                }
                if (successor == nullptr || entry.createSeq > successor->createSeq) {
                    successor = &entry;
                }
            }
            if (successor != nullptr) {
                DiagRecord record;
                record.kind = static_cast<uint8_t>(DiagKind::kSfCompetition);
                record.event = static_cast<uint8_t>(DiagEvent::kSfSuperseded);
                record.state = static_cast<uint8_t>(event.newState);
                record.detail = static_cast<uint8_t>(successor->state);
                record.instanceId = event.instanceId;
                record.generation = event.generation;
                record.sessTagId = event.sessTagId;
                record.a = successor->instanceId;
                record.b = successor->generation;
                uint64_t ignored = 0;
                appendLocked(record, &ignored);
            }
            mLiveSf.erase(event.instanceId);
            break;
        }
        default:
            break;
    }
}

std::vector<AdpfDiag::SessionEntry> AdpfDiag::snapshotSfLocked() const {
    std::vector<SessionEntry> entries;
    entries.reserve(mLiveSf.size());
    for (const auto &[instanceId, entry] : mLiveSf) {
        entries.push_back(entry);
    }
    std::sort(entries.begin(), entries.end(),
              [](const SessionEntry &left, const SessionEntry &right) {
                  return left.instanceId < right.instanceId;
              });
    return entries;
}

std::string AdpfDiag::formatError(const char *code) const {
    std::string out = "{\"ok\":false,\"error\":\"";
    out.append(code);
    out.append("\"}\n");
    return out;
}

std::string AdpfDiag::formatReceipt(const std::string &windowId, const char *marker,
                                    uint64_t markerSeq, int64_t boottimeNs, uint64_t dropCount,
                                    const std::vector<SessionEntry> &sessions) const {
    std::string out = "{\"ok\":true,\"schema\":";
    out.append(StringPrintf("%" PRIu32, kSchemaVersion));
    out.append(",\"window_id\":");
    appendJsonString(&out, windowId);
    out.append(",\"marker\":");
    appendJsonString(&out, marker);
    out.append(",\"hal_instance_id\":");
    appendJsonString(&out, mHalInstanceId);
    out.append(StringPrintf(",\"marker_seq\":%" PRIu64 ",\"boottime_ns\":%" PRId64
                            ",\"drop_count\":%" PRIu64 ",\"active_sf_instances\":[",
                            markerSeq, boottimeNs, dropCount));
    bool first = true;
    for (const auto &entry : sessions) {
        if (!first) {
            out.append(",");
        }
        first = false;
        out.append(StringPrintf("{\"instance_id\":%" PRId64 ",\"generation\":%" PRIu32
                                ",\"state\":",
                                entry.instanceId, entry.generation));
        appendJsonString(&out, toString(entry.state));
        out.append("}");
    }
    out.append("]}\n");
    return out;
}

std::string AdpfDiag::formatRecord(const DiagRecord &record, uint64_t seq,
                                   const std::vector<std::string> &names) const {
    std::string out;
    out.append(StringPrintf("{\"seq\":%" PRIu64 ",\"kind\":", seq));
    const DiagKind kind = static_cast<DiagKind>(record.kind);
    const char *kindName = "unknown";
    switch (kind) {
        case DiagKind::kDecision:
            kindName = "decision";
            break;
        case DiagKind::kLifecycle:
            kindName = "lifecycle";
            break;
        case DiagKind::kSfCompetition:
            kindName = "sf_competition";
            break;
        case DiagKind::kReportUnclassified:
            kindName = "report_unclassified";
            break;
        case DiagKind::kMarker:
            kindName = "marker";
            break;
    }
    appendJsonString(&out, kindName);
    out.append(",\"rec\":{\"schema_version\":");
    out.append(StringPrintf("%" PRIu32, kSchemaVersion));
    switch (kind) {
        case DiagKind::kDecision:
            out.append(",\"boot_id\":");
            appendJsonString(&out, mBootId);
            out.append(StringPrintf(",\"hal_process_start_id\":%" PRIu64, mProcessStartTicks));
            out.append(StringPrintf(",\"session_instance_id\":%" PRId64, record.instanceId));
            out.append(StringPrintf(",\"session_generation\":%" PRIu32, record.generation));
            out.append(StringPrintf(",\"report_sequence\":%" PRIu64, record.reportSeq));
            out.append(StringPrintf(",\"item_index\":%" PRIu32, record.itemIndex));
            out.append(StringPrintf(",\"decision_boottime_ns\":%" PRId64, record.boottimeNs));
            out.append(",\"process_tag\":");
            appendJsonString(&out, lookupName(names, record.procTagId));
            out.append(",\"session_tag\":");
            appendJsonString(&out, lookupName(names, record.sessTagId));
            out.append(",\"profile_name\":");
            appendJsonString(&out, lookupName(names, record.profileId));
            out.append(StringPrintf(",\"target_duration_ns_used\":%" PRId64, record.a));
            out.append(StringPrintf(",\"reported_duration_ns_used\":%" PRId64, record.b));
            out.append(",\"factor_used_bits\":");
            appendJsonBits(&out, record.hasFactor != 0, record.factorBits);
            out.append(",\"factor_used\":");
            appendJsonNullableNumber(&out, record.hasFactor != 0,
                                     doubleFromBits(record.factorBits));
            out.append(",\"effective_threshold_used\":");
            appendJsonNumber(&out, record.thresholdUs);
            out.append(",\"missed_actual\":");
            out.append(record.detail != 0 ? "true" : "false");
            out.append(",\"session_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.state)));
            break;
        case DiagKind::kLifecycle:
            out.append(",\"boot_id\":");
            appendJsonString(&out, mBootId);
            out.append(StringPrintf(",\"hal_process_start_id\":%" PRIu64, mProcessStartTicks));
            out.append(StringPrintf(",\"session_instance_id\":%" PRId64, record.instanceId));
            out.append(StringPrintf(",\"session_generation\":%" PRIu32, record.generation));
            out.append(",\"event\":");
            appendJsonString(&out, toString(static_cast<DiagEvent>(record.event)));
            out.append(StringPrintf(",\"boottime_ns\":%" PRId64, record.boottimeNs));
            out.append(",\"process_tag\":");
            appendJsonString(&out, lookupName(names, record.procTagId));
            out.append(",\"session_tag\":");
            appendJsonString(&out, lookupName(names, record.sessTagId));
            out.append(",\"profile_name\":");
            appendJsonString(&out, lookupName(names, record.profileId));
            out.append(",\"current_profile_name\":");
            appendJsonString(&out, lookupName(names, record.profileId2));
            out.append(",\"factor_used_bits\":");
            appendJsonBits(&out, record.hasFactor != 0, record.factorBits);
            out.append(",\"factor_used\":");
            appendJsonNullableNumber(&out, record.hasFactor != 0,
                                     doubleFromBits(record.factorBits));
            out.append(",\"new_profile_factor_bits\":");
            appendJsonBits(&out, record.b != 0, static_cast<uint64_t>(record.a));
            out.append(",\"prev_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.detail)));
            out.append(",\"session_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.state)));
            out.append(",\"origin\":");
            appendJsonString(&out, toString(static_cast<DiagOrigin>(record.origin)));
            out.append(",\"reason\":");
            if (record.reason == static_cast<uint8_t>(DiagReason::kNone)) {
                out.append("null");
            } else {
                appendJsonString(&out, toString(static_cast<DiagReason>(record.reason)));
            }
            out.append(StringPrintf(",\"report_sequence\":%" PRIu64, record.reportSeq));
            break;
        case DiagKind::kSfCompetition:
            out.append(",\"boot_id\":");
            appendJsonString(&out, mBootId);
            out.append(StringPrintf(",\"hal_process_start_id\":%" PRIu64, mProcessStartTicks));
            out.append(StringPrintf(",\"session_instance_id\":%" PRId64, record.instanceId));
            out.append(StringPrintf(",\"session_generation\":%" PRIu32, record.generation));
            out.append(",\"event\":");
            appendJsonString(&out, toString(static_cast<DiagEvent>(record.event)));
            out.append(StringPrintf(",\"boottime_ns\":%" PRId64, record.boottimeNs));
            out.append(",\"session_tag\":");
            appendJsonString(&out, lookupName(names, record.sessTagId));
            out.append(StringPrintf(",\"other_instance_id\":%" PRId64, record.a));
            out.append(StringPrintf(",\"other_generation\":%" PRId64, record.b));
            out.append(",\"other_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.detail)));
            out.append(",\"session_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.state)));
            break;
        case DiagKind::kReportUnclassified:
            out.append(",\"boot_id\":");
            appendJsonString(&out, mBootId);
            out.append(StringPrintf(",\"hal_process_start_id\":%" PRIu64, mProcessStartTicks));
            out.append(StringPrintf(",\"session_instance_id\":%" PRId64, record.instanceId));
            out.append(StringPrintf(",\"session_generation\":%" PRIu32, record.generation));
            out.append(StringPrintf(",\"report_sequence\":%" PRIu64, record.reportSeq));
            out.append(StringPrintf(",\"batch_size\":%" PRId64, record.a));
            out.append(",\"reason\":");
            appendJsonString(&out, toString(static_cast<DiagReason>(record.reason)));
            out.append(",\"process_tag\":");
            appendJsonString(&out, lookupName(names, record.procTagId));
            out.append(",\"session_tag\":");
            appendJsonString(&out, lookupName(names, record.sessTagId));
            out.append(",\"session_state\":");
            appendJsonString(&out, toString(static_cast<DiagState>(record.state)));
            break;
        case DiagKind::kMarker:
            out.append(",\"marker\":");
            appendJsonString(&out, record.event == static_cast<uint8_t>(DiagEvent::kBegin)
                                           ? "begin"
                                           : "end");
            out.append(",\"window_id\":");
            appendJsonString(&out, joinWindowId(static_cast<uint64_t>(record.a),
                                                static_cast<uint64_t>(record.b)));
            out.append(StringPrintf(",\"boottime_ns\":%" PRId64, record.boottimeNs));
            break;
    }
    out.append(StringPrintf(",\"dropped_record_count\":%" PRIu64, record.dropCount));
    out.append("}}");
    return out;
}

std::string AdpfDiag::overwrittenLocked(const Window &window) const {
    std::string out = "{\"ok\":false,\"error\":\"OVERWRITTEN\",\"schema\":";
    out.append(StringPrintf("%" PRIu32, kSchemaVersion));
    out.append(",\"window_id\":");
    appendJsonString(&out, window.id);
    out.append(",\"hal_instance_id\":");
    appendJsonString(&out, mHalInstanceId);
    out.append(StringPrintf(",\"first_seq\":%" PRIu64 ",\"last_seq\":%" PRIu64
                            ",\"oldest_retained_seq\":%" PRIu64 ",\"drop_count\":%" PRIu64 "}\n",
                            window.beginSeq, window.endSeq, oldestRetainedLocked(), mOverwritten));
    return out;
}

std::string AdpfDiag::begin(const std::string &windowId) {
    std::scoped_lock lock{mMutex};
    if (!mRing) {
        return formatError("ring_unavailable");
    }
    if (mOpenWindow.has_value()) {
        return formatError("window_already_open");
    }
    if (mUsedWindowIds.count(windowId) != 0) {
        return formatError("duplicate_window_id");
    }
    uint64_t high = 0;
    uint64_t low = 0;
    splitWindowId(windowId, &high, &low);
    DiagRecord record;
    record.kind = static_cast<uint8_t>(DiagKind::kMarker);
    record.event = static_cast<uint8_t>(DiagEvent::kBegin);
    record.a = static_cast<int64_t>(high);
    record.b = static_cast<int64_t>(low);
    uint64_t seq = 0;
    if (!appendLocked(record, &seq)) {
        return formatError("ring_unavailable");
    }
    std::vector<SessionEntry> sessions = snapshotSfLocked();
    Window window;
    window.id = windowId;
    window.beginSeq = seq;
    window.beginNs = record.boottimeNs;
    window.beginDropCount = record.dropCount;
    window.beginSf = sessions;
    mOpenWindow = window;
    mUsedWindowIds.insert(windowId);
    return formatReceipt(windowId, "begin", seq, record.boottimeNs, record.dropCount, sessions);
}

std::string AdpfDiag::end(const std::string &windowId) {
    std::scoped_lock lock{mMutex};
    if (!mOpenWindow.has_value() || mOpenWindow->id != windowId) {
        return formatError("no_matching_begin");
    }
    DiagRecord record;
    record.kind = static_cast<uint8_t>(DiagKind::kMarker);
    record.event = static_cast<uint8_t>(DiagEvent::kEnd);
    uint64_t high = 0;
    uint64_t low = 0;
    splitWindowId(windowId, &high, &low);
    record.a = static_cast<int64_t>(high);
    record.b = static_cast<int64_t>(low);
    uint64_t seq = 0;
    if (!appendLocked(record, &seq)) {
        return formatError("ring_unavailable");
    }
    std::vector<SessionEntry> sessions = snapshotSfLocked();
    Window window = *mOpenWindow;
    window.endSeq = seq;
    window.endNs = record.boottimeNs;
    window.endDropCount = record.dropCount;
    window.endSf = sessions;
    mOpenWindow.reset();
    mClosedWindows.push_back(window);
    while (mClosedWindows.size() > kMaxClosedWindows) {
        mClosedWindows.pop_front();
    }
    return formatReceipt(windowId, "end", seq, record.boottimeNs, record.dropCount, sessions);
}

std::string AdpfDiag::read(const std::string &windowId) {
    Window window;
    {
        std::scoped_lock lock{mMutex};
        if (mOpenWindow.has_value() && mOpenWindow->id == windowId) {
            return formatError("window_not_ended");
        }
        const Window *found = nullptr;
        for (const auto &candidate : mClosedWindows) {
            if (candidate.id == windowId) {
                found = &candidate;
            }
        }
        if (found == nullptr) {
            return formatError("unknown_window");
        }
        window = *found;
        if (mRing == nullptr || window.beginSeq < oldestRetainedLocked()) {
            return overwrittenLocked(window);
        }
    }

    std::vector<DiagRecord> records;
    records.reserve(static_cast<size_t>(window.endSeq - window.beginSeq + 1));
    uint64_t cursor = window.beginSeq;
    while (cursor <= window.endSeq) {
        uint64_t last = std::min<uint64_t>(window.endSeq, cursor + kReadChunk - 1);
        std::scoped_lock lock{mMutex};
        if (cursor < oldestRetainedLocked()) {
            return overwrittenLocked(window);
        }
        for (uint64_t seq = cursor; seq <= last; ++seq) {
            records.push_back(mRing[(seq - 1) % kRingCapacity]);
        }
        cursor = last + 1;
    }

    std::vector<std::string> names;
    uint64_t dropCount = 0;
    uint64_t failedInserts = 0;
    uint64_t nameOverflow = 0;
    {
        std::scoped_lock lock{mMutex};
        if (window.beginSeq < oldestRetainedLocked()) {
            return overwrittenLocked(window);
        }
        names = mNames;
        dropCount = mOverwritten;
        failedInserts = mFailedInserts;
        nameOverflow = mNameOverflow;
    }

    std::string out = "{\"ok\":true,\"schema\":";
    out.append(StringPrintf("%" PRIu32, kSchemaVersion));
    out.append(",\"window_id\":");
    appendJsonString(&out, window.id);
    out.append(",\"hal_instance_id\":");
    appendJsonString(&out, mHalInstanceId);
    out.append(StringPrintf(",\"first_seq\":%" PRIu64 ",\"last_seq\":%" PRIu64
                            ",\"record_count\":%" PRIu64 ",\"drop_count\":%" PRIu64
                            ",\"failed_insert_count\":%" PRIu64
                            ",\"name_overflow_count\":%" PRIu64
                            ",\"begin_boottime_ns\":%" PRId64 ",\"end_boottime_ns\":%" PRId64,
                            window.beginSeq, window.endSeq, window.endSeq - window.beginSeq + 1,
                            dropCount, failedInserts, nameOverflow, window.beginNs, window.endNs));
    const auto appendEntries = [&out](const char *key, const std::vector<SessionEntry> &entries) {
        out.append(",\"");
        out.append(key);
        out.append("\":[");
        bool first = true;
        for (const auto &entry : entries) {
            if (!first) {
                out.append(",");
            }
            first = false;
            out.append(StringPrintf("{\"instance_id\":%" PRId64 ",\"generation\":%" PRIu32
                                    ",\"state\":",
                                    entry.instanceId, entry.generation));
            appendJsonString(&out, toString(entry.state));
            out.append("}");
        }
        out.append("]");
    };
    appendEntries("begin_active_sf_instances", window.beginSf);
    appendEntries("end_active_sf_instances", window.endSf);
    out.append(",\"records\":[");
    bool first = true;
    uint64_t seq = window.beginSeq;
    for (const auto &record : records) {
        if (!first) {
            out.append(",");
        }
        first = false;
        out.append(formatRecord(record, seq, names));
        ++seq;
    }
    out.append("]}\n");
    return out;
}

void AdpfDiag::handleDumpCommand(int fd, const char *const *args, uint32_t numArgs) {
    std::string out;
    if (numArgs != 2 || args == nullptr || args[0] == nullptr || args[1] == nullptr) {
        out = formatError("bad_arg_count");
    } else {
        const std::string verb(args[0]);
        const std::string windowId(args[1]);
        if (verb != "begin" && verb != "end" && verb != "read") {
            out = formatError("unknown_subcommand");
        } else if (windowId.size() != kWindowIdLength || !isLowerHex(windowId)) {
            out = formatError("malformed_window_id");
        } else if (verb == "begin") {
            out = begin(windowId);
        } else if (verb == "end") {
            out = end(windowId);
        } else {
            out = read(windowId);
        }
    }

    SigpipeGuard guard;
    size_t offset = 0;
    while (offset < out.size()) {
        size_t chunk = std::min(kWriteChunk, out.size() - offset);
        size_t written = 0;
        while (written < chunk) {
            ssize_t count = write(fd, out.data() + offset + written, chunk - written);
            if (count > 0) {
                written += static_cast<size_t>(count);
                continue;
            }
            if (count < 0 && errno == EINTR) {
                continue;
            }
            return;
        }
        offset += chunk;
    }
}

}  // namespace aidl::google::hardware::power::impl::pixel
