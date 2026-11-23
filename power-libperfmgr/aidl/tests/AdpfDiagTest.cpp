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

#include <gtest/gtest.h>

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "aidl/AdpfDiag.h"
#include "aidl/SessionRecords.h"

namespace aidl {
namespace google {
namespace hardware {
namespace power {
namespace impl {
namespace pixel {

namespace {

const char kBootId[] = "11111111-2222-3333-4444-555555555555";

// handleDumpCommand writes to a plain fd, so a temporary file doubles as the
// dump sink without the deadlock a full pipe would cause.
class DumpSink {
  public:
    DumpSink() {
        mFile = tmpfile();
        EXPECT_NE(mFile, nullptr);
    }
    ~DumpSink() {
        if (mFile != nullptr) {
            fclose(mFile);
        }
    }
    int fd() const { return fileno(mFile); }

    std::string take() {
        EXPECT_EQ(fseeko(mFile, 0, SEEK_SET), 0);
        std::string out;
        char buffer[4096];
        while (true) {
            size_t count = fread(buffer, 1, sizeof(buffer), mFile);
            out.append(buffer, count);
            if (count < sizeof(buffer)) {
                break;
            }
        }
        return out;
    }

  private:
    FILE *mFile{nullptr};
};

size_t countOf(const std::string &text, const std::string &needle) {
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos;
         pos = text.find(needle, pos + needle.size())) {
        ++count;
    }
    return count;
}

// Returns the raw text of a scalar value of "key", or "" when absent.
std::string field(const std::string &json, const std::string &key) {
    const std::string prefix = "\"" + key + "\":";
    size_t pos = json.find(prefix);
    if (pos == std::string::npos) {
        return "";
    }
    pos += prefix.size();
    if (pos >= json.size()) {
        return "";
    }
    if (json[pos] == '"') {
        size_t end = json.find('"', pos + 1);
        return end == std::string::npos ? "" : json.substr(pos + 1, end - pos - 1);
    }
    size_t end = pos;
    while (end < json.size() && json[end] != ',' && json[end] != '}') {
        ++end;
    }
    return json.substr(pos, end - pos);
}

}  // namespace

class AdpfDiagTest : public ::testing::Test {
  protected:
    void SetUp() override {
        char path[] = "/tmp/adpfdiag-bootid-XXXXXX";
        int fd = mkstemp(path);
        ASSERT_GE(fd, 0);
        ASSERT_EQ(write(fd, kBootId, sizeof(kBootId) - 1), static_cast<ssize_t>(sizeof(kBootId) - 1));
        close(fd);
        mBootIdPath = path;
        mSources.bootIdPath = mBootIdPath.c_str();
        mSources.statPath = "/proc/self/stat";
    }

    void TearDown() override { unlink(mBootIdPath.c_str()); }

    std::string command(AdpfDiag &diag, const std::vector<std::string> &args) {
        std::vector<const char *> raw;
        for (const auto &arg : args) {
            raw.push_back(arg.c_str());
        }
        DumpSink sink;
        diag.handleDumpCommand(sink.fd(), raw.data(), raw.size());
        return sink.take();
    }

    std::string begin(AdpfDiag &diag, const std::string &windowId) {
        return command(diag, {"begin", windowId});
    }
    std::string end(AdpfDiag &diag, const std::string &windowId) {
        return command(diag, {"end", windowId});
    }
    std::string read(AdpfDiag &diag, const std::string &windowId) {
        return command(diag, {"read", windowId});
    }

    void unclassified(AdpfDiag &diag, int64_t instanceId = 7, uint32_t batchSize = 1,
                      DiagReason reason = DiagReason::kHboostOff,
                      uint64_t reportSeq = 1) {
        diag.appendUnclassified(instanceId, 1, reportSeq, diag.internName("top-app"),
                                diag.internName("APP"), batchSize, reason, DiagState::kActive);
    }

    void decision(AdpfDiag &diag, int64_t reportedNs, int64_t targetNs, double factor,
                  size_t itemIndex = 0) {
        AdpfDiagDecisionContext context;
        context.sink = &diag;
        context.instanceId = 11;
        context.generation = 2;
        context.reportSeq = 3;
        context.procTagId = diag.internName("top-app");
        context.sessTagId = diag.internName("APP");
        context.profileId = diag.internName("Default");
        context.state = DiagState::kActive;
        diag.appendDecision(context, itemIndex, reportedNs, targetNs, factor,
                            (targetNs / 1000) * factor, reportedNs / 1000 > (targetNs / 1000) * factor);
    }

    std::string mBootIdPath;
    AdpfDiag::Sources mSources;
};

TEST_F(AdpfDiagTest, BeginAndEndReturnIdentityAndMarker) {
    AdpfDiag diag(mSources);
    const std::string windowId = "0123456789abcdef0123456789abcdef";

    const std::string beginReceipt = begin(diag, windowId);
    EXPECT_NE(beginReceipt.find("\"ok\":true"), std::string::npos) << beginReceipt;
    EXPECT_EQ(field(beginReceipt, "window_id"), windowId);
    EXPECT_EQ(field(beginReceipt, "marker"), "begin");
    EXPECT_EQ(field(beginReceipt, "marker_seq"), "1");
    EXPECT_EQ(field(beginReceipt, "hal_instance_id").substr(0, sizeof(kBootId) - 1),
              std::string(kBootId));
    EXPECT_EQ(field(beginReceipt, "schema"), "1");

    const std::string endReceipt = end(diag, windowId);
    EXPECT_NE(endReceipt.find("\"ok\":true"), std::string::npos) << endReceipt;
    EXPECT_EQ(field(endReceipt, "marker"), "end");
    EXPECT_EQ(field(endReceipt, "marker_seq"), "2");
}

TEST_F(AdpfDiagTest, ReadReturnsRetainedRecordsInSequenceOrder) {
    AdpfDiag diag(mSources);
    const std::string windowId = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    begin(diag, windowId);
    unclassified(diag, 7, 4, DiagReason::kHboostOff, 1);
    decision(diag, 9'000'000, 8'000'000, 1.0, 0);
    decision(diag, 6'000'000, 8'000'000, 1.0, 1);
    end(diag, windowId);

    const std::string dump = read(diag, windowId);
    ASSERT_NE(dump.find("\"ok\":true"), std::string::npos) << dump;
    EXPECT_EQ(field(dump, "first_seq"), "1");
    EXPECT_EQ(field(dump, "last_seq"), "5");
    EXPECT_EQ(field(dump, "record_count"), "5");
    EXPECT_EQ(countOf(dump, "\"kind\":\"marker\""), 2u);
    EXPECT_EQ(countOf(dump, "\"kind\":\"decision\""), 2u);
    EXPECT_EQ(countOf(dump, "\"kind\":\"report_unclassified\""), 1u);

    // Sequence numbers are strictly increasing in the emitted order.
    size_t pos = 0;
    int64_t previous = 0;
    while ((pos = dump.find("\"seq\":", pos)) != std::string::npos) {
        pos += 6;
        int64_t seq = std::stoll(dump.substr(pos, dump.find(',', pos) - pos));
        EXPECT_GT(seq, previous);
        previous = seq;
    }

    // The two decisions disagree about "missed", from the same comparison.
    EXPECT_EQ(countOf(dump, "\"missed_actual\":true"), 1u);
    EXPECT_EQ(countOf(dump, "\"missed_actual\":false"), 1u);
    EXPECT_NE(dump.find("\"factor_used\":1,"), std::string::npos);
    EXPECT_NE(dump.find("\"effective_threshold_used\":8000"), std::string::npos) << dump;
    EXPECT_EQ(field(dump, "batch_size"), "4");
    EXPECT_NE(dump.find("\"reason\":\"hboost_off\""), std::string::npos) << dump;
}

TEST_F(AdpfDiagTest, DecisionRecordsTheComparisonItWasGiven) {
    AdpfDiag diag(mSources);
    const std::string windowId = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    begin(diag, windowId);
    // Exactly at the threshold is not a miss; the comparison is strict and the
    // operands are microsecond integers, so a sub-microsecond excess is not seen.
    decision(diag, 9'600'000, 8'000'000, 1.2, 0);
    decision(diag, 9'600'999, 8'000'000, 1.2, 1);
    decision(diag, 9'601'000, 8'000'000, 1.2, 2);
    end(diag, windowId);

    const std::string dump = read(diag, windowId);
    ASSERT_NE(dump.find("\"ok\":true"), std::string::npos) << dump;
    const size_t threshold = dump.find("\"effective_threshold_used\":");
    ASSERT_NE(threshold, std::string::npos);
    // 9600 us exactly for the first two, 9601 us classifies as missed.
    EXPECT_NE(dump.find("\"effective_threshold_used\":9600"), std::string::npos) << dump;
    EXPECT_EQ(countOf(dump, "\"missed_actual\":true"), 1u);
    EXPECT_EQ(countOf(dump, "\"item_index\":0"), 1u);
    EXPECT_NE(dump.find("\"item_index\":2"), std::string::npos);
    EXPECT_NE(dump.find("\"profile_name\":\"Default\""), std::string::npos) << dump;
    EXPECT_NE(dump.find("\"process_tag\":\"top-app\""), std::string::npos) << dump;
    EXPECT_NE(dump.find("\"session_state\":\"ACTIVE\""), std::string::npos) << dump;
}

TEST_F(AdpfDiagTest, SessionRecordsEmitsOneDecisionPerItem) {
    AdpfDiag diag(mSources);
    const std::string windowId = "cccccccccccccccccccccccccccccccc";
    begin(diag, windowId);

    SessionRecords records(64, 1.2, "Default");
    AdpfDiagDecisionContext context;
    context.sink = &diag;
    context.instanceId = 21;
    context.sessTagId = diag.internName("APP");
    context.state = DiagState::kActive;

    std::vector<WorkDuration> durations(3);
    durations[0].durationNanos = 9'600'000;
    durations[1].durationNanos = 9'600'999;
    durations[2].durationNanos = 9'601'000;
    FrameTimingMetrics metrics;
    records.addReportedDurations(durations, 8'000'000, metrics, false, &context);
    end(diag, windowId);

    const std::string dump = read(diag, windowId);
    ASSERT_NE(dump.find("\"ok\":true"), std::string::npos) << dump;
    EXPECT_EQ(countOf(dump, "\"kind\":\"decision\""), 3u);
    EXPECT_EQ(countOf(dump, "\"missed_actual\":true"), 1u);
    EXPECT_NE(dump.find("\"item_index\":2"), std::string::npos);
    // The record names the profile whose factor the records were built with.
    EXPECT_NE(dump.find("\"factor_used\":1.2"), std::string::npos) << dump;
}

TEST_F(AdpfDiagTest, ArgumentErrorsChangeNoState) {
    AdpfDiag diag(mSources);
    const std::string windowId = "dddddddddddddddddddddddddddddddd";

    EXPECT_NE(command(diag, {"begin"}).find("\"error\":\"bad_arg_count\""), std::string::npos);
    EXPECT_NE(command(diag, {}).find("\"error\":\"bad_arg_count\""), std::string::npos);
    EXPECT_NE(command(diag, {"begin", windowId, "extra"}).find("\"error\":\"bad_arg_count\""),
              std::string::npos);
    EXPECT_NE(command(diag, {"frobnicate", windowId}).find("\"error\":\"unknown_subcommand\""),
              std::string::npos);
    EXPECT_NE(command(diag, {"begin", "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD"})
                      .find("\"error\":\"malformed_window_id\""),
              std::string::npos);
    EXPECT_NE(command(diag, {"begin", "short"}).find("\"error\":\"malformed_window_id\""),
              std::string::npos);
    EXPECT_NE(end(diag, windowId).find("\"error\":\"no_matching_begin\""), std::string::npos);

    // None of the rejected commands opened a window or consumed the id.
    EXPECT_NE(begin(diag, windowId).find("\"ok\":true"), std::string::npos);
    EXPECT_NE(begin(diag, windowId).find("\"error\":\"window_already_open\""), std::string::npos);
    EXPECT_NE(read(diag, windowId).find("\"error\":\"window_not_ended\""), std::string::npos);
    EXPECT_NE(end(diag, "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee")
                      .find("\"error\":\"no_matching_begin\""),
              std::string::npos);
    EXPECT_NE(end(diag, windowId).find("\"ok\":true"), std::string::npos);
    EXPECT_NE(begin(diag, windowId).find("\"error\":\"duplicate_window_id\""), std::string::npos);
}

TEST_F(AdpfDiagTest, MissingRingReportsUnavailable) {
    mSources.allocateRing = false;
    AdpfDiag diag(mSources);
    EXPECT_NE(begin(diag, "ffffffffffffffffffffffffffffffff").find("\"error\":\"ring_unavailable\""),
              std::string::npos);
}

TEST_F(AdpfDiagTest, OverwrittenWindowReportsTheLoss) {
    AdpfDiag diag(mSources);
    const std::string windowId = "1234567890abcdef1234567890abcdef";
    begin(diag, windowId);
    for (uint64_t i = 0; i < AdpfDiag::kRingCapacity + 1; ++i) {
        unclassified(diag, 7, 1, DiagReason::kHboostOff, i);
    }
    end(diag, windowId);

    const std::string dump = read(diag, windowId);
    EXPECT_NE(dump.find("\"ok\":false"), std::string::npos) << dump.substr(0, 200);
    EXPECT_NE(dump.find("\"error\":\"OVERWRITTEN\""), std::string::npos);
    EXPECT_EQ(field(dump, "first_seq"), "1");
    // begin + 65537 appends + end reach seq 65539, so the oldest record still
    // held is 65539 - 65536 + 1 = 4.
    EXPECT_EQ(field(dump, "oldest_retained_seq"), "4");
    EXPECT_GT(std::stoull(field(dump, "oldest_retained_seq")), 1u);
}

TEST_F(AdpfDiagTest, SurfaceFlingerSessionsAreMirroredForTheWindow) {
    AdpfDiag diag(mSources);
    const std::string windowId = "abcdefabcdefabcdefabcdefabcdefab";
    const std::string receipt = begin(diag, windowId);
    EXPECT_NE(receipt.find("\"active_sf_instances\":[]"), std::string::npos) << receipt;

    AdpfDiagLifecycle create;
    create.event = DiagEvent::kCreate;
    create.origin = DiagOrigin::kClient;
    create.instanceId = 41;
    create.generation = 1;
    create.newState = DiagState::kActive;
    create.isSurfaceFlinger = true;
    diag.appendLifecycle(create);

    AdpfDiagLifecycle competing = create;
    competing.instanceId = 42;
    competing.generation = 3;
    diag.appendLifecycle(competing);

    AdpfDiagLifecycle close = create;
    close.event = DiagEvent::kClose;
    close.instanceId = 41;
    close.generation = 1;
    close.previousState = DiagState::kActive;
    close.newState = DiagState::kClosed;
    diag.appendLifecycle(close);

    end(diag, windowId);
    const std::string dump = read(diag, windowId);
    ASSERT_NE(dump.find("\"ok\":true"), std::string::npos) << dump;
    EXPECT_EQ(countOf(dump, "\"kind\":\"sf_competition\""), 2u);
    EXPECT_NE(dump.find("\"event\":\"SF_CREATE_WITH_LIVE\""), std::string::npos) << dump;
    EXPECT_NE(dump.find("\"event\":\"SF_SUPERSEDED\""), std::string::npos) << dump;
    EXPECT_NE(dump.find("\"other_instance_id\":41"), std::string::npos);
    EXPECT_NE(dump.find("\"other_instance_id\":42"), std::string::npos);
    EXPECT_NE(dump.find("\"end_active_sf_instances\":[{\"instance_id\":42"), std::string::npos)
            << dump;
}

TEST_F(AdpfDiagTest, HalRestartGetsANewIdentityAndHidesTheOldWindows) {
    AdpfDiag first(mSources);
    const std::string windowId = "0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f0f";
    const std::string firstReceipt = begin(first, windowId);
    end(first, windowId);

    // A second process reads its own boot id and start time, so its instance id
    // differs and a window opened before it restarted is unknown to it.
    AdpfDiag::Sources restarted = mSources;
    restarted.statPath = "/proc/1/stat";
    AdpfDiag second(restarted);
    EXPECT_NE(field(firstReceipt, "hal_instance_id"), second.halInstanceId());
    EXPECT_NE(read(second, windowId).find("\"error\":\"unknown_window\""), std::string::npos);
}

TEST_F(AdpfDiagTest, OneAppendCostsMicroseconds) {
    AdpfDiag diag(mSources);
    const std::string windowId = "1a2b3c4d5e6f708192a3b4c5d6e7f809";
    begin(diag, windowId);
    constexpr int kIterations = 200000;
    const auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        decision(diag, 8'000'000, 8'000'000, 1.0, static_cast<size_t>(i));
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    const double perAppendUs =
            std::chrono::duration<double, std::micro>(elapsed).count() / kIterations;
    // One classified item costs one fixed-size record under a leaf mutex; a
    // report path that took materially longer than this would show a defect.
    EXPECT_LT(perAppendUs, 5.0) << perAppendUs << " us per append";
}

TEST_F(AdpfDiagTest, LifecycleRecordsCoverEveryTransition) {
    AdpfDiag diag(mSources);
    const std::string windowId = "99999999999999999999999999999999";
    begin(diag, windowId);

    AdpfDiagLifecycle lifecycle;
    lifecycle.instanceId = 51;
    lifecycle.generation = 4;
    lifecycle.procTagId = diag.internName("top-app");
    lifecycle.sessTagId = diag.internName("APP");
    lifecycle.profileId = diag.internName("Default");
    lifecycle.hasFactor = true;
    lifecycle.factor = 1.25;
    lifecycle.previousState = DiagState::kActive;
    lifecycle.newState = DiagState::kPaused;
    lifecycle.event = DiagEvent::kPause;
    lifecycle.origin = DiagOrigin::kClient;
    diag.appendLifecycle(lifecycle);

    lifecycle.event = DiagEvent::kRejected;
    lifecycle.reason = DiagReason::kAlreadyActive;
    lifecycle.previousState = DiagState::kActive;
    lifecycle.newState = DiagState::kActive;
    diag.appendLifecycle(lifecycle);

    lifecycle.event = DiagEvent::kProfileSwap;
    lifecycle.reason = DiagReason::kNone;
    lifecycle.origin = DiagOrigin::kProfileCallback;
    lifecycle.newHasFactor = true;
    lifecycle.newProfileFactor = 1.6;
    lifecycle.currentProfileId = diag.internName("Sustained");
    diag.appendLifecycle(lifecycle);
    end(diag, windowId);

    const std::string dump = read(diag, windowId);
    ASSERT_NE(dump.find("\"ok\":true"), std::string::npos) << dump;
    EXPECT_EQ(countOf(dump, "\"kind\":\"lifecycle\""), 3u);
    EXPECT_NE(dump.find("\"event\":\"PAUSE\""), std::string::npos);
    EXPECT_NE(dump.find("\"prev_state\":\"ACTIVE\""), std::string::npos);
    EXPECT_NE(dump.find("\"event\":\"REJECTED\""), std::string::npos);
    EXPECT_NE(dump.find("\"reason\":\"already_active\""), std::string::npos);
    EXPECT_NE(dump.find("\"event\":\"PROFILE_SWAP\""), std::string::npos);
    EXPECT_NE(dump.find("\"current_profile_name\":\"Sustained\""), std::string::npos) << dump;
    EXPECT_NE(dump.find("\"new_profile_factor_bits\":"), std::string::npos);
    EXPECT_NE(dump.find("\"origin\":\"profile_callback\""), std::string::npos);
    EXPECT_NE(dump.find("\"factor_used\":1.25"), std::string::npos) << dump;
}

}  // namespace pixel
}  // namespace impl
}  // namespace power
}  // namespace hardware
}  // namespace google
}  // namespace aidl
