#include "channel/channel.hpp"
#include "channel/model/channel_model.hpp"

namespace openCREST {

Channel::Channel(const ChannelConfig&   config,
                 const CalibrationData& source_cal,
                 const CalibrationData& receiver_cal,
                 const TransducerSpec&  source_transducer,
                 const TransducerSpec&  receiver_transducer,
                 float                  receive_boost_db)
    : config_(config)
    , sample_rate_(source_cal.adc_sampling_rate)
{
    ChannelBuildContext ctx;
    ctx.source_cal          = source_cal;
    ctx.receiver_cal        = receiver_cal;
    ctx.source_transducer   = source_transducer;
    ctx.receiver_transducer = receiver_transducer;
    ctx.receive_boost_db    = receive_boost_db;
    ctx.sample_rate         = sample_rate_;

    renderer_ = find_channel_model(config_.mode).make_renderer(config_, ctx);
}

Channel::~Channel() = default;

void Channel::on_message_start(PairBuffer& pair_buffer,
                               size_t      inter_message_gap_samples,
                               bool        absolute_first_origin) {
    renderer_->on_message_start();
    pair_buffer.begin_message(inter_message_gap_samples, absolute_first_origin);
}

void Channel::on_message_end(PairBuffer& pair_buffer) {
    renderer_->on_message_end(pair_buffer);
}

size_t Channel::process(const float* samples, size_t count,
                        PairBuffer& pair_buffer) {
    return renderer_->process(samples, count, pair_buffer);
}

size_t Channel::input_needed_for_batch() const {
    return renderer_->input_needed_for_batch();
}

size_t Channel::max_tap_delta_samples() const {
    return renderer_->max_tap_delta_samples();
}

size_t Channel::base_delay_samples() const {
    return renderer_->base_delay_samples();
}

double Channel::current_doppler_ratio() const {
    return renderer_->current_doppler_ratio();
}

} // namespace openCREST
