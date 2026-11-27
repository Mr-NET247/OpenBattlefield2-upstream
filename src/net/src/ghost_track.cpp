#include "obf2/net/ghost_track.h"

#include <algorithm>
#include <cmath>

namespace obf2::net::bf2 {

std::array<float, 4> interpolateGhostRotation(const std::array<float, 4>& a,
                                              const std::array<float, 4>& b, float fraction) {
  const float sign = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3] < 0 ? -1.0f : 1.0f;
  std::array<float, 4> blended;
  float norm = 0.0f;
  for (std::size_t i = 0; i < blended.size(); ++i) {
    blended[i] = std::lerp(a[i], sign * b[i], fraction);
    norm += blended[i] * blended[i];
  }
  if (norm <= 0.0f) return a;
  const float scale = 1.0f / std::sqrt(norm);
  for (float& component : blended) component *= scale;
  return blended;
}

GhostSample& GhostTrack::push(double timeMs, bool hasPosition, bool hasRotation) {
  if (count_ > 0 && ring_[head_].timeMs == timeMs) {
    ring_[head_].hasPosition |= hasPosition;
    ring_[head_].hasRotation |= hasRotation;
    return ring_[head_];
  }
  const GhostSample previous = count_ > 0 ? ring_[head_] : GhostSample{};
  head_ = count_ > 0 ? (head_ + 1) % kSlots : 0;
  if (count_ < kSlots) ++count_;
  ring_[head_] = previous;
  ring_[head_].timeMs = timeMs;
  ring_[head_].hasPosition = hasPosition || count_ == 1;
  ring_[head_].hasRotation = hasRotation;
  return ring_[head_];
}

const GhostSample* GhostTrack::newest() const { return count_ > 0 ? &ring_[head_] : nullptr; }

std::optional<GhostPose> GhostTrack::leadVehicle(float milliseconds, float maximumSpeed) const {
  const auto* latest = newest();
  if (!latest) return std::nullopt;
  GhostPose pose{latest->position, latest->bodyYaw, latest->pitch, latest->aimYaw,
                 latest->rotation, latest->stance, GhostPrediction::Newest};
  if (count_ < 2) return pose;
  const auto& previous = slot(head_ + kSlots - 1);
  const float gap = latest->timeMs - previous.timeMs;
  if (gap <= 0.0f || gap > 4.0f * kGhostTickMs) return pose;  // not measured
  const float lead = std::clamp(milliseconds, 0.0f, 100.0f);   // not measured
  const auto traveled = latest->position - previous.position;
  if (length(traveled) * 1000.0f > gap * maximumSpeed) return pose;
  pose.position = latest->position + traveled * (lead / gap);
  if (latest->hasRotation && latest->rotation && previous.hasRotation && previous.rotation) {
    const auto& a = *previous.rotation;
    const auto& b = *latest->rotation;
    const float similarity = std::abs(a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);
    const float radians = 2.0f * std::acos(std::min(similarity, 1.0f));
    if (radians <= gap * 0.003f) {  // not measured
      pose.rotation = interpolateGhostRotation(a, b, 1.0f + lead / gap);
    }
  }
  if (lead > 0.0f) pose.mode = GhostPrediction::Extrapolated;
  return pose;
}

std::optional<GhostPose> GhostTrack::poseAt(double nowMs, float interpolationMs,
                                            float extrapolationMs, bool holdAtGaps,
                                            float maximumSpeed) const {
  const GhostSample* last = newest();
  if (last == nullptr) return std::nullopt;

  const auto samplePose = [](const GhostSample& sample) {
    GhostPose pose;
    pose.position = sample.position;
    pose.bodyYaw = sample.bodyYaw;
    pose.pitch = sample.pitch;
    pose.aimYaw = sample.aimYaw;
    pose.rotation = sample.rotation;
    pose.stance = sample.stance;
    return pose;
  };
  GhostPose pose = samplePose(*last);

  const double at = nowMs - interpolationMs;
  if (holdAtGaps) {
    // Position and orientation can be sent on different ticks. A carried
    // value is a baseline, not a new observation of that component.
    struct Samples {
      const GhostSample* before = nullptr;
      const GhostSample* earlier = nullptr;
      const GhostSample* later = nullptr;

      double gap() const { return later && earlier ? later->timeMs - earlier->timeMs : 0.0; }
    };
    const auto bracket = [&](auto include) {
      Samples out;
      for (std::size_t i = 0; i < count_; ++i) {
        const auto& sample = slot(head_ + kSlots - (count_ - 1) + i);
        if (!include(sample)) continue;
        if (sample.timeMs > at) {
          out.later = &sample;
          break;
        }
        out.before = out.earlier;
        out.earlier = &sample;
      }
      return out;
    };
    const auto position = bracket([](const GhostSample& s) { return s.hasPosition; });
    const auto angles = bracket([](const GhostSample&) { return true; });
    const auto rotation = bracket([](const GhostSample& s) { return s.hasRotation && s.rotation; });
    const auto fraction = [at](const Samples& s) {
      return static_cast<float>(std::clamp((at - s.earlier->timeMs) / s.gap(), 0.0, 1.0));
    };

    if (const auto* sample = angles.earlier ? angles.earlier : angles.later) {
      // Partial records carry the last known pose even after its original
      // observation ages out. Use that baseline until an observed field
      // reaches the playback time.
      pose = samplePose(*sample);
    }
    if (angles.gap() > 0.0f && angles.gap() < kGhostMaxGapMs) {
      const float f = fraction(angles);
      pose.bodyYaw += std::remainder(angles.later->bodyYaw - pose.bodyYaw, 360.0f) * f;
      pose.pitch = std::lerp(pose.pitch, angles.later->pitch, f);
      pose.aimYaw = std::lerp(pose.aimYaw, angles.later->aimYaw, f);
    }

    if (position.earlier) {
      const auto& anchor = *position.earlier;
      pose.position = anchor.position;
      const float gap = position.gap();
      if (gap > 0.0f && gap < kGhostMaxGapMs &&
          length(position.later->position - anchor.position) <= 1.0f + gap * maximumSpeed * 0.001f) {
        pose.position = anchor.position + (position.later->position - anchor.position) * fraction(position);
        pose.mode = GhostPrediction::Interpolated;
      } else if (!position.later && !angles.later && extrapolationMs > 0.0f) {
        // A subsequent update with an explicit zero velocity supersedes a
        // moving position sample. Extrapolation stays anchored to the position.
        Vec3f velocity{};
        if (last->velocity) {
          velocity = *last->velocity;
        } else if (position.before && last->timeMs - anchor.timeMs <= 2.0f * kGhostTickMs) {
          const float ms = anchor.timeMs - position.before->timeMs;
          if (ms < kGhostMaxGapMs) {
            velocity = (anchor.position - position.before->position) * (1000.0f / ms);
          }
        }
        const float over = static_cast<float>(std::min<double>(at - anchor.timeMs, extrapolationMs));
        const float speed = length(velocity);
        if (over > 0.0f && speed > 0.0001f && speed <= maximumSpeed) {
          pose.position = anchor.position + velocity * (over * 0.001f);
          pose.mode = GhostPrediction::Extrapolated;
        }
      }
    }

    if (rotation.earlier) {
      pose.rotation = rotation.earlier->rotation;
      if (rotation.gap() > 0.0f && rotation.gap() < kGhostMaxGapMs) {
        pose.rotation = interpolateGhostRotation(*pose.rotation, *rotation.later->rotation,
                                                  fraction(rotation));
      }
    }
    return pose;
  }
  if (at <= last->timeMs) {
    // 0x62bb10: needs two updates and a moment not before the oldest.
    if (count_ < 2) return pose;
    const std::size_t oldest = head_ + kSlots - (count_ - 1);
    if (slot(oldest).timeMs > at) return pose;
    for (std::size_t i = 1; i < count_; ++i) {
      const GhostSample& later = slot(oldest + i);
      if (later.timeMs < at) continue;
      const GhostSample& earlier = slot(oldest + i - 1);
      const float gap = later.timeMs - earlier.timeMs;
      if (!(gap > 0.0f && gap < kGhostMaxGapMs)) return pose;
      const float f = gap > 1.0f ? (at - earlier.timeMs) / gap : 1.0f;
      pose.position = earlier.position + (later.position - earlier.position) * f;
      pose.bodyYaw = earlier.bodyYaw + std::remainder(later.bodyYaw - earlier.bodyYaw, 360.0f) * f;
      pose.pitch = earlier.pitch + (later.pitch - earlier.pitch) * f;
      pose.aimYaw = earlier.aimYaw + (later.aimYaw - earlier.aimYaw) * f;
      pose.stance = (f < 1.0f ? earlier : later).stance;
      if (earlier.rotation && later.rotation) {
        pose.rotation = interpolateGhostRotation(*earlier.rotation, *later.rotation, f);
      }
      pose.mode = GhostPrediction::Interpolated;
      return pose;
    }
    return pose;
  }

  if (extrapolationMs <= 0.0f) return pose;
  const float over = at - last->timeMs;
  if (over >= extrapolationMs) return pose;
  // A soldier carries its velocity; a simple object's is the difference of its
  // last two positions over their gap (0x62f7d0).
  Vec3f velocity{};
  if (last->velocity) {
    velocity = *last->velocity;
  } else if (count_ >= 2) {
    const GhostSample& before = slot(head_ + kSlots - 1);
    const float gap = last->timeMs - before.timeMs;
    if (gap > 0.0f) velocity = (last->position - before.position) * (1000.0f / gap);
  }
  pose.position = last->position + velocity * (over * 0.001f);
  pose.mode = GhostPrediction::Extrapolated;
  return pose;
}

}  // namespace obf2::net::bf2
