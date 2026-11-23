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

#pragma once

#include <android-base/thread_annotations.h>

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

namespace aidl::google::hardware::power::impl::pixel {

// ADPF jank-classification diagnostic ring.
//
// One record is appended per classified item, at the comparison itself, from the
// operands the comparison uses (SessionRecords::addReportedDurations):
//
//   reportedUs = (double)(int32_t)(reported_duration_ns_used / 1000)   // truncates toward zero
//   effective_threshold_used = (double)(target_duration_ns_used / 1000) * factor_used
//                                                                    // double, MICROSECONDS
//   missed_actual = reportedUs > effective_threshold_used            // strict
//
// target_duration_ns_used is AppHintDesc::targetNs, already scaled by
// TargetTimeFactor; reported_duration_ns_used is WorkDuration::durationNanos.
// factor_used is SessionRecords::kJankCheckTimeFactor, copied when the
// SessionRecords object was built from the profile named by profile_name, so a
// later profile swap does not change it; PROFILE_SWAP records carry the profile
// that arrived and the factor still in use.
//
// Lock order, outermost first:
//
//   ChannelGroup::mGroupMutex                      (FMQ report path only)
//     -> PowerHintSession::mPowerHintSessionLock   (session lifecycle + decision lock)
//          -> AdpfDiag::mMutex                     (leaf)
//
// AdpfDiag::mMutex is a leaf: no other lock is acquired and no descriptor is
// written while it is held. Marker commands take only AdpfDiag::mMutex, and its
// snapshot is this component's own mirror of the live SurfaceFlinger sessions,
// which every transition updates inside the same critical section that appends
// its lifecycle record.

enum class DiagKind : uint8_t {
    kDecision = 1,
    kLifecycle,
    kSfCompetition,
    kReportUnclassified,
    kMarker,
};

enum class DiagState : uint8_t {
    kNone = 0,
    kActive,
    kPaused,
    kClosed,
    kDestroyed,
};

enum class DiagEvent : uint8_t {
    kCreate = 1,
    kPause,
    kResume,
    kClose,
    kDestroy,
    kProfileSwap,
    kRejected,
    kSfCreateWithLive,
    kSfSuperseded,
    kBegin,
    kEnd,
};

enum class DiagOrigin : uint8_t {
    kNone = 0,
    kClient,
    kDestructor,
    kProfileCallback,
};

enum class DiagReason : uint8_t {
    kNone = 0,
    kClosed,
    kNoTarget,
    kEmptyBatch,
    kPaused,
    kPidOff,
    kHboostOff,
    kNotActive,
    kAlreadyActive,
};

const char *toString(DiagState state);
const char *toString(DiagEvent event);
const char *toString(DiagOrigin origin);
const char *toString(DiagReason reason);

struct DiagRecord {
    int64_t boottimeNs{0};
    int64_t instanceId{0};
    uint64_t reportSeq{0};
    // Per-kind payload; see AdpfDiag.cpp's formatters.
    int64_t a{0};
    int64_t b{0};
    uint64_t factorBits{0};
    double thresholdUs{0};
    uint64_t dropCount{0};
    uint32_t generation{0};
    uint32_t itemIndex{0};
    uint16_t profileId{0};
    uint16_t profileId2{0};
    uint16_t procTagId{0};
    uint16_t sessTagId{0};
    uint8_t kind{0};
    uint8_t state{0};
    uint8_t detail{0};  // decision: missed_actual; competition: other session's state
    uint8_t event{0};
    uint8_t origin{0};
    uint8_t reason{0};
    uint8_t hasFactor{0};
    uint8_t reserved{0};
};

static_assert(sizeof(DiagRecord) == 88, "the ring stores fixed 88-byte records");
static_assert(std::is_trivially_copyable_v<DiagRecord>);

// Identity and state a decision record needs, built by the session that holds
// the decision lock, so the append neither reads that session's state nor
// recomputes anything.
struct AdpfDiagDecisionContext {
    class AdpfDiag *sink{nullptr};
    int64_t instanceId{0};
    uint32_t generation{0};
    uint64_t reportSeq{0};
    uint16_t procTagId{0};
    uint16_t sessTagId{0};
    uint16_t profileId{0};
    DiagState state{DiagState::kNone};
};

struct AdpfDiagLifecycle {
    DiagEvent event{DiagEvent::kCreate};
    DiagOrigin origin{DiagOrigin::kNone};
    DiagReason reason{DiagReason::kNone};
    int64_t instanceId{0};
    uint32_t generation{0};
    uint64_t lastReportSeq{0};
    DiagState previousState{DiagState::kNone};
    DiagState newState{DiagState::kNone};
    uint16_t procTagId{0};
    uint16_t sessTagId{0};
    uint16_t profileId{0};
    uint16_t currentProfileId{0};
    bool isSurfaceFlinger{false};
    bool hasFactor{false};
    double factor{0};
    bool newHasFactor{false};
    double newProfileFactor{0};
};

class AdpfDiag {
  public:
    static constexpr uint32_t kSchemaVersion = 1;
    // The consumer reads windows of at most this many records, so the ring holds
    // one window's worth and reports anything older as OVERWRITTEN.
    static constexpr uint64_t kRingCapacity = 65536;
    static constexpr uint16_t kNoId = 0;
    static constexpr uint16_t kIdOverflow = 0xFFFF;
    // Sized for every tag and profile name a device interning table can hold.
    static constexpr size_t kMaxNames = 1024;
    static constexpr size_t kMaxClosedWindows = 16;
    static constexpr size_t kReadChunk = 4096;

    struct Sources {
        // Fixed by the kernel: the boot id file and the process stat format.
        const char *bootIdPath = "/proc/sys/kernel/random/boot_id";
        const char *statPath = "/proc/self/stat";
        bool allocateRing = true;
    };

    explicit AdpfDiag(const Sources &sources);
    AdpfDiag() : AdpfDiag(Sources{}) {}
    AdpfDiag(const AdpfDiag &) = delete;
    AdpfDiag &operator=(const AdpfDiag &) = delete;

    // Interns a process tag, session tag or profile name.
    uint16_t internName(const std::string &name);

    void appendDecision(const AdpfDiagDecisionContext &context, uint32_t itemIndex,
                        int64_t reportedDurationNs, int64_t targetDurationNs, double factorUsed,
                        double thresholdUs, bool missed) EXCLUDES(mMutex);
    void appendLifecycle(const AdpfDiagLifecycle &record) EXCLUDES(mMutex);
    void appendUnclassified(int64_t instanceId, uint32_t generation, uint64_t reportSeq,
                            uint16_t procTagId, uint16_t sessTagId, uint32_t batchSize,
                            DiagReason reason, DiagState state) EXCLUDES(mMutex);

    // Handles "adpfdiag <begin|end|read> <window-id>" and writes exactly one JSON
    // object to fd. Any other argument vector writes one error object and
    // changes no state.
    void handleDumpCommand(int fd, const char *const *args, uint32_t numArgs) EXCLUDES(mMutex);

    const std::string &halInstanceId() const { return mHalInstanceId; }

  private:
    struct SessionEntry {
        int64_t instanceId{0};
        uint32_t generation{0};
        DiagState state{DiagState::kNone};
        uint64_t createSeq{0};
    };

    struct Window {
        std::string id;
        uint64_t beginSeq{0};
        uint64_t endSeq{0};
        int64_t beginNs{0};
        int64_t endNs{0};
        uint64_t beginDropCount{0};
        uint64_t endDropCount{0};
        std::vector<SessionEntry> beginSf;
        std::vector<SessionEntry> endSf;
    };

    bool appendLocked(DiagRecord &record, uint64_t *seqOut) REQUIRES(mMutex);
    std::vector<SessionEntry> snapshotSfLocked() const REQUIRES(mMutex);
    void updateSfLocked(const AdpfDiagLifecycle &record, uint64_t createSeq)
            REQUIRES(mMutex);
    uint64_t oldestRetainedLocked() const REQUIRES(mMutex);

    std::string begin(const std::string &windowId);
    std::string end(const std::string &windowId);
    std::string read(const std::string &windowId);
    std::string overwrittenLocked(const Window &window) const REQUIRES(mMutex);

    std::string formatRecord(const DiagRecord &record, uint64_t seq,
                             const std::vector<std::string> &names) const;
    std::string formatReceipt(const std::string &windowId, const char *marker, uint64_t markerSeq,
                              int64_t boottimeNs, uint64_t dropCount,
                              const std::vector<SessionEntry> &sf) const;
    std::string formatError(const char *code) const;

    std::mutex mMutex;
    const std::string mBootId;
    const uint64_t mProcessStartTicks;
    const int32_t mPid;
    const std::string mHalInstanceId;
    const std::unique_ptr<DiagRecord[]> mRing;

    uint64_t mNextSeq GUARDED_BY(mMutex) = 1;  // sequence 0 means "none"
    uint64_t mOverwritten GUARDED_BY(mMutex) = 0;
    uint64_t mFailedInserts GUARDED_BY(mMutex) = 0;
    uint64_t mNameOverflow GUARDED_BY(mMutex) = 0;
    std::vector<std::string> mNames GUARDED_BY(mMutex){std::string()};
    std::map<int64_t, SessionEntry> mLiveSf GUARDED_BY(mMutex);
    std::optional<Window> mOpenWindow GUARDED_BY(mMutex);
    std::deque<Window> mClosedWindows GUARDED_BY(mMutex);
    std::unordered_set<std::string> mUsedWindowIds GUARDED_BY(mMutex);
};

}  // namespace aidl::google::hardware::power::impl::pixel
