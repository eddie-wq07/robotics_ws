export function preparationDisplay(info) {
  const stage = info.preparation_stage || 'Loading checked drawing';
  const progress = info.preparation_progress;
  if (!progress || !Number.isFinite(progress.fraction)) return { text: `${stage}…`, value: null };
  const value = Math.max(0, Math.min(1, progress.fraction));
  const seconds = progress.eta_seconds;
  const rounded = Math.ceil(seconds);
  const eta = Number.isFinite(seconds) && seconds >= 0
    ? `~${rounded >= 60 ? `${Math.floor(rounded / 60)}m ` : ''}${rounded % 60}s remaining`
    : 'estimating time remaining…';
  return { text: `${stage} · ${Math.floor(value * 100)}% · ${eta}`, value };
}
