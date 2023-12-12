import process from "process";

// Environment variables
import dotenv from "dotenv";
dotenv.config();

// Config Interface
interface IConfig {
	// Access control
	whitelistedPhoneNumbers: string[];
	whitelistedEnabled: boolean;
	// OpenAI
	openAIModel: string;
	openAIAPIKeys: string[];
	maxModelTokens: number;
	prePrompt: string | undefined;


	// Groupchats
	groupchatsEnabled: boolean;


	// Voice transcription & Text-to-Speech
	ttsEnabled: boolean;
	transcriptionEnabled: boolean;
	transcriptionLanguage: string;
}

// Config
export const config: IConfig = {
	whitelistedPhoneNumbers: process.env.WHITELISTED_PHONE_NUMBERS?.split(",") || [],
	whitelistedEnabled: getEnvBooleanWithDefault("WHITELISTED_ENABLED", false),

	openAIAPIKeys: (process.env.OPENAI_API_KEYS || process.env.OPENAI_API_KEY || "").split(",").filter((key) => !!key), // Default: []
	openAIModel: process.env.OPENAI_GPT_MODEL || "gpt-3.5-turbo", // Default: gpt-3.5-turbo
	maxModelTokens: getEnvMaxModelTokens(), // Default: 4096
	prePrompt: process.env.PRE_PROMPT, // Default: undefined


	// Groupchats
	groupchatsEnabled: getEnvBooleanWithDefault("GROUPCHATS_ENABLED", false), // Default: false

	

	// Text-to-Speech
	ttsEnabled: getEnvBooleanWithDefault("TTS_ENABLED", false), // Default: false


	// Transcription
	transcriptionEnabled: getEnvBooleanWithDefault("TRANSCRIPTION_ENABLED", false), // Default: false
	transcriptionLanguage: process.env.TRANSCRIPTION_LANGUAGE || "" // Default: null
};

/**
 * Get the max model tokens from the environment variable
 * @returns The max model tokens from the environment variable or 4096
 */
function getEnvMaxModelTokens() {
	const envValue = process.env.MAX_MODEL_TOKENS;
	if (envValue == undefined || envValue == "") {
		return 4096;
	}

	return parseInt(envValue);
}

/**
 * Get an environment variable as a boolean with a default value
 * @param key The environment variable key
 * @param defaultValue The default value
 * @returns The value of the environment variable or the default value
 */
function getEnvBooleanWithDefault(key: string, defaultValue: boolean): boolean {
	const envValue = process.env[key]?.toLowerCase();
	if (envValue == undefined || envValue == "") {
		return defaultValue;
	}

	return envValue == "true";
}

/**
 * Get the blacklist categories for prompt moderation from the environment variable
 * @returns Blacklisted categories for prompt moderation
 */
function getEnvPromptModerationBlacklistedCategories(): string[] {
	const envValue = process.env.PROMPT_MODERATION_BLACKLISTED_CATEGORIES;
	if (envValue == undefined || envValue == "") {
		return ["hate", "hate/threatening", "self-harm", "sexual", "sexual/minors", "violence", "violence/graphic"];
	} else {
		return JSON.parse(envValue.replace(/'/g, '"'));
	}
}

export default config;