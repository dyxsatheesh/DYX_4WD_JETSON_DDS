"""Spoken phrases for rover voice announcements (English, Tamil, Hindi).

Pure data + rendering, no I/O. Every phrase key must exist in every
language; ``render`` falls back to English for an unknown language.

Tamil/Hindi wording for the pre-existing phrases is taken from the 3WD
NRP_ROS backend; the LASER / pause-reason phrases are new and should be
reviewed by a native speaker before wide rollout.
"""

from __future__ import annotations

from typing import Any

LANGUAGES: dict[str, str] = {
    "en": "English",
    "ta": "Tamil",
    "hi": "Hindi",
}

DEFAULT_LANGUAGE = "en"

PHRASES: dict[str, dict[str, str]] = {
    # ------------------------------------------------------------ startup
    "startup": {
        "en": "Way to Mark ready.",
        "ta": "வே டு மார்க் தயார்.",
        "hi": "वे टू मार्क तैयार है.",
    },
    "good_morning": {
        "en": "Good morning!",
        "ta": "காலை வணக்கம்!",
        "hi": "सुप्रभात!",
    },
    "good_afternoon": {
        "en": "Good afternoon!",
        "ta": "மதிய வணக்கம்!",
        "hi": "नमस्कार!",
    },
    "good_evening": {
        "en": "Good evening!",
        "ta": "மாலை வணக்கம்!",
        "hi": "शुभ संध्या!",
    },
    # ------------------------------------------------------------ mission
    "mission_ready": {
        "en": "Mission ready.",
        "ta": "பணி தயார்.",
        "hi": "मिशन तैयार है.",
    },
    "mission_started": {
        "en": "Mission started.",
        "ta": "பணி தொடங்கியது.",
        "hi": "मिशन शुरू हो गया.",
    },
    "heading_to_target": {
        "en": "Heading to target {n}.",
        "ta": "இலக்கு {n} க்கு செல்கிறோம்.",
        "hi": "लक्ष्य {n} की तरफ जा रहे हैं.",
    },
    "target_done": {
        "en": "Target {n} done.",
        "ta": "இலக்கு {n} முடிந்தது.",
        "hi": "लक्ष्य {n} पूरा हो गया.",
    },
    "target_laser_mark": {
        "en": "Target {n}. Laser on. Mark here.",
        "ta": "இலக்கு {n}. லேசர் இயக்கத்தில். இங்கே குறியிடுங்கள்.",
        "hi": "लक्ष्य {n}. लेज़र चालू. यहाँ निशान लगाएँ.",
    },
    "target_laser_do_not_mark": {
        "en": "Target {n}. Outside tolerance. Do not mark.",
        "ta": "இலக்கு {n}. துல்லிய வரம்புக்கு வெளியே. குறியிட வேண்டாம்.",
        "hi": "लक्ष्य {n}. सटीकता सीमा से बाहर. निशान न लगाएँ.",
    },
    "target_failed": {
        "en": "Target {n} failed.",
        "ta": "இலக்கு {n} தோல்வி.",
        "hi": "लक्ष्य {n} फेल हो गया.",
    },
    "target_skipped": {
        "en": "Target {n} skipped.",
        "ta": "இலக்கு {n} தவிர்க்கப்பட்டது.",
        "hi": "लक्ष्य {n} स्किप हो गया.",
    },
    "mission_paused": {
        "en": "Mission paused.",
        "ta": "பணி இடைநிறுத்தப்பட்டது.",
        "hi": "मिशन रुक गया.",
    },
    "paused_rtk_float": {
        "en": "RTK float. Mission paused.",
        "ta": "ஆர்.டி.கே ஃப்ளோட். பணி இடைநிறுத்தப்பட்டது.",
        "hi": "RTK फ्लोट. मिशन रुक गया.",
    },
    "paused_rtk_lost": {
        "en": "RTK lost. Mission paused.",
        "ta": "ஆர்.டி.கே இழந்தது. பணி இடைநிறுத்தப்பட்டது.",
        "hi": "RTK खो गया. मिशन रुक गया.",
    },
    "paused_vehicle_control": {
        "en": "Vehicle control lost. Mission paused.",
        "ta": "வாகன கட்டுப்பாடு இழந்தது. பணி இடைநிறுத்தப்பட்டது.",
        "hi": "वाहन नियंत्रण खो गया. मिशन रुक गया.",
    },
    "paused_position": {
        "en": "Position data lost. Mission paused.",
        "ta": "நிலை தரவு இழந்தது. பணி இடைநிறுத்தப்பட்டது.",
        "hi": "स्थिति डेटा खो गया. मिशन रुक गया.",
    },
    "emergency_stop": {
        "en": "Emergency stop.",
        "ta": "அவசர நிறுத்தம்.",
        "hi": "आपातकालीन स्टॉप.",
    },
    "mission_resumed": {
        "en": "Mission resumed.",
        "ta": "பணி மீண்டும் தொடங்கப்பட்டது.",
        "hi": "मिशन फिर से शुरू.",
    },
    "mission_stopped": {
        "en": "Mission stopped.",
        "ta": "பணி நிறுத்தப்பட்டது.",
        "hi": "मिशन रोक दिया गया.",
    },
    "mission_completed": {
        "en": "Mission completed.",
        "ta": "பணி முடிந்தது.",
        "hi": "मिशन पूरा हुआ.",
    },
    "mission_error": {
        "en": "Mission error.",
        "ta": "பணி பிழை.",
        "hi": "मिशन में त्रुटि.",
    },
    # ------------------------------------------------------------ RTK
    "rtk_fixed": {
        "en": "RTK fixed.",
        "ta": "ஆர்.டி.கே சரி.",
        "hi": "RTK फिक्स मिल गया.",
    },
    "rtk_float": {
        "en": "RTK float.",
        "ta": "ஆர்.டி.கே ஃப்ளோட்.",
        "hi": "RTK फ्लोट.",
    },
    "rtk_lost": {
        "en": "RTK lost. GPS only.",
        "ta": "ஆர்.டி.கே இழந்தது. ஜி.பி.எஸ் மட்டும்.",
        "hi": "RTK खो गया. केवल GPS.",
    },
    # ------------------------------------------------------------ battery
    "battery_low": {
        "en": "Battery low, {pct} percent.",
        "ta": "பேட்டரி குறைவு, {pct} சதவீதம்.",
        "hi": "बैटरी कम है, {pct} प्रतिशत बाकी.",
    },
    "battery_critical": {
        "en": "Warning! Battery critical, {pct} percent.",
        "ta": "எச்சரிக்கை! பேட்டரி மிகக் குறைவு, {pct} சதவீதம்.",
        "hi": "चेतावनी! बैटरी गंभीर, {pct} प्रतिशत बाकी.",
    },
    # ------------------------------------------------------------ vehicle link
    "connection_lost": {
        "en": "Warning! Vehicle connection lost.",
        "ta": "எச்சரிக்கை! வாகன இணைப்பு துண்டிக்கப்பட்டது.",
        "hi": "चेतावनी! वाहन कनेक्शन टूट गया.",
    },
    "connection_restored": {
        "en": "Vehicle connected.",
        "ta": "வாகனம் இணைக்கப்பட்டது.",
        "hi": "वाहन जुड़ गया.",
    },
    # ------------------------------------------------------------ settings
    "language_set": {
        "en": "Language set to English.",
        "ta": "மொழி தமிழுக்கு மாற்றப்பட்டது.",
        "hi": "भाषा हिंदी में सेट.",
    },
    "voice_enabled": {
        "en": "Voice enabled.",
        "ta": "குரல் இயக்கப்பட்டது.",
        "hi": "आवाज़ चालू.",
    },
    "voice_changed": {
        "en": "Voice changed.",
        "ta": "குரல் மாற்றப்பட்டது.",
        "hi": "आवाज़ बदल दी गई.",
    },
    "voice_test": {
        "en": "Voice test.",
        "ta": "குரல் சோதனை.",
        "hi": "आवाज़ परीक्षण.",
    },
}


def normalize_language(language: Any) -> str:
    code = str(language or "").strip().lower()
    return code if code in LANGUAGES else DEFAULT_LANGUAGE


def render(key: str, language: str, **params: Any) -> str:
    """Return the phrase for key in language (English if missing)."""
    variants = PHRASES[key]
    template = variants.get(normalize_language(language)) or variants[DEFAULT_LANGUAGE]
    return template.format(**params)


def greeting_key(hour: int) -> str:
    if hour < 12:
        return "good_morning"
    if hour < 17:
        return "good_afternoon"
    return "good_evening"
