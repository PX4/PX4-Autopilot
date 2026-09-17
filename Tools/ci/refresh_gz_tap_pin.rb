# frozen_string_literal: true

# Move Tools/setup/gz-tap-pin.txt to the newest osrf/simulation commit whose
# Gazebo formulae all have a bottle Homebrew would pour on this machine.
#
# Runs under Homebrew's Ruby so it can use Homebrew's own tap, formula and
# bottle logic:
#
#     brew ruby Tools/ci/refresh_gz_tap_pin.rb
#
# macos.sh --sim-tools pins the osrf/simulation tap to gz-tap-pin.txt so
# Gazebo pours from bottles even while OSRF has them pulled (see that file).
# The pin is a point in time, so this walks the tap's first-parent history
# from origin HEAD back to the current pin, checks each commit out, and asks
# Homebrew whether every osrf/simulation formula macos.sh installs, plus
# their osrf/simulation runtime dependencies, carries a bottle for this
# host. Each bottle tarball is then HEAD-requested so a bottle block whose
# tarball is gone does not count. The first commit that passes becomes the
# new pin. The tap checkout is restored on exit either way.
#
# A formula that has no bottle at the current pin either is not required to
# have one: in practice that is the gz-harmonic meta-formula, which OSRF
# never bottles and which builds in seconds. Everything that poured at the
# pin has to keep pouring.
#
# The formula names are read from macos.sh so there is no second list to
# keep in sync. The bottle tag match is Homebrew's own, which on macOS also
# accepts a bottle built on an older macOS of the same arch, so a commit
# that passes on the oldest macOS CI runs on passes on the newer ones too.
#
# Formula#bottle is deliberately not used: `brew ruby` runs in a child of
# the brew process, which has already consumed the HOMEBREW_*_DEFAULT_PREFIX
# variables Homebrew derives its default cellar from, so here every bottle
# without an explicit cellar looks like it was built in a foreign prefix.
# The tag match and the bottle URL do not depend on that.
#
# Exits 0 whether or not a newer bottled commit exists; only errors exit
# non-zero. Under GitHub Actions the result also lands in $GITHUB_OUTPUT as
# needs_bump=true|false and old_pin, plus new_pin and new_pin_short when
# needs_bump is true. (brew scrubs the environment but forwards GITHUB_*
# whenever CI is set, which GitHub Actions does.)

require "English"
require "net/http"

# Everything lives here; RefreshGzTapPin.run at the bottom is the entry point.
module RefreshGzTapPin
  TAP_NAME = "osrf/simulation"
  ROOT_DIR = Pathname(__dir__).parent.parent.freeze
  PIN_FILE = (ROOT_DIR/"Tools/setup/gz-tap-pin.txt").freeze
  MACOS_SH = (ROOT_DIR/"Tools/setup/macos.sh").freeze
  ROOT_FORMULA = %r{#{Regexp.escape(TAP_NAME)}/[A-Za-z0-9@._+-]+}

  module_function

  def log(message)
    puts "[refresh_gz_tap_pin] #{message}"
  end

  def die(message)
    warn "[refresh_gz_tap_pin] ERROR: #{message}"
    exit 1
  end

  def output(key, value)
    log "#{key}=#{value}"
    File.open(ENV.fetch("GITHUB_OUTPUT"), "a") { |f| f.puts "#{key}=#{value}" } if ENV["GITHUB_OUTPUT"]
  end

  # Run git in the tap clone and return its stripped output. A failure is
  # fatal unless fail: false, in which case it returns nil.
  def git(tap, *args, fail: true)
    out = Utils.popen_read("git", "-C", tap.path.to_s, *args, err: :out).strip
    return out if $CHILD_STATUS.success?

    die "git #{args.join(" ")} failed in #{tap.path}: #{out}" if fail
    nil
  end

  # Every osrf/simulation formula in the runtime closure of the roots, as
  # loaded from the tap's current checkout.
  def tap_formulae(tap, roots)
    Formulary.clear_cache
    tap.clear_cache
    formulae = {}
    roots.each do |name|
      root = Formulary.factory(name)
      deps = root.recursive_dependencies do |_, dep|
        Dependable::PRUNE if dep.build? || dep.test? || dep.optional?
      end
      ([root] + deps.map(&:to_formula)).each do |f|
        formulae[f.full_name] = f if f.tap&.name == TAP_NAME
      end
    end
    formulae
  end

  # The bottle this host would pour for the formula, or nil.
  def bottle_for(formula)
    formula.bottle_for_tag(Utils::Bottles.tag) if formula.pour_bottle?
  end

  def served?(url)
    uri = URI(url)
    Net::HTTP.start(uri.host, uri.port, use_ssl: uri.scheme == "https") do |http|
      http.head(uri.request_uri).is_a?(Net::HTTPSuccess)
    end
  end

  # Whether every formula that poured at the pin still pours at the checked
  # out commit and every bottle tarball is still served. One line per formula.
  def bottled_here?(tap, roots, exempt)
    ok = true
    tap_formulae(tap, roots).each_value do |f|
      bottle = bottle_for(f)
      state = if bottle.nil? && exempt.include?(f.full_name)
        "no bottle, not required"
      elsif bottle.nil?
        ok = false
        "no bottle"
      elsif served?(bottle.url)
        "bottled"
      else
        ok = false
        "tarball missing"
      end
      puts "    #{state.ljust(24)} #{f.full_name} #{f.pkg_version}"
    end
    ok
  end

  def run
    # The osrf/simulation formulae macos.sh installs. Read from the script
    # so this never drifts from PX4_SIM_BREW_PACKAGES.
    roots = MACOS_SH.read.scan(ROOT_FORMULA).uniq.sort
    die "macos.sh installs nothing from #{TAP_NAME}" if roots.empty?

    old_pin = PIN_FILE.read.lines.map(&:strip).grep_v(/\A#/).join
    die "no commit SHA in #{PIN_FILE}" unless old_pin.match?(/\A[0-9a-f]{40}\z/)

    tap = Tap.fetch(TAP_NAME)
    tap.install(quiet: true) unless tap.installed?
    # Homebrew 6.0+ refuses to load formulae from untrusted taps; same guard
    # as macos.sh.
    Homebrew::Trust.trust!(:tap, tap) if defined?(Homebrew::Trust) && !Homebrew::Trust.trusted_tap?(tap)

    die "#{tap.path} has local changes, refusing to touch it" unless git(tap, "status", "--porcelain").empty?
    orig_ref = git(tap, "symbolic-ref", "--quiet", "--short", "HEAD", fail: false) || git(tap, "rev-parse", "HEAD")

    new_pin = nil
    begin
      git(tap, "fetch", "--quiet", "--unshallow") if git(tap, "rev-parse", "--is-shallow-repository") == "true"
      git(tap, "fetch", "--quiet", "origin")
      git(tap, "remote", "set-head", "origin", "--auto")
      tap_head = git(tap, "rev-parse", "origin/HEAD")
      if git(tap, "merge-base", "--is-ancestor", old_pin, "origin/HEAD", fail: false).nil?
        die "pin #{old_pin} is not an ancestor of #{TAP_NAME} HEAD #{tap_head}, bump it by hand"
      end

      output "old_pin", old_pin
      git(tap, "checkout", "--quiet", "--detach", old_pin)
      exempt = tap_formulae(tap, roots).each_value.reject { |f| bottle_for(f) }.map(&:full_name)
      log "not bottled at the pin either, not required: #{exempt.join(" ")}" unless exempt.empty?

      candidates = git(tap, "rev-list", "--first-parent", "#{old_pin}..origin/HEAD").split
      log "#{TAP_NAME} HEAD #{tap_head} is #{candidates.size} commit(s) ahead of the pin, " \
          "checking newest first for #{roots.join(" ")}"
      new_pin = candidates.find do |sha|
        log git(tap, "log", "-1", "--format=%h %cs %s", sha)
        git(tap, "checkout", "--quiet", "--detach", sha)
        bottled_here?(tap, roots, exempt)
      end
    ensure
      git(tap, "checkout", "--quiet", orig_ref)
    end

    if new_pin.nil?
      log "no commit newer than the pin is fully bottled here, keeping #{old_pin}"
      output "needs_bump", "false"
      return
    end

    PIN_FILE.write(PIN_FILE.read.sub(old_pin, new_pin))
    behind = git(tap, "rev-list", "--count", "#{new_pin}..origin/HEAD")
    log "pinned #{TAP_NAME} to #{new_pin}, #{behind} commit(s) behind HEAD"
    output "needs_bump", "true"
    output "new_pin", new_pin
    output "new_pin_short", new_pin[0, 12]
  end
end

RefreshGzTapPin.run
