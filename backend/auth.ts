import { init } from "better-auth";
import { postgresAdapter } from "better-auth/adapters/postgres";

export const auth = init({
  secret: process.env.BETTER_AUTH_SECRET || "your-secret-key-change-this",
  database: {
    provider: "postgresql",
    url: process.env.DATABASE_URL || "your-db-url",
  },
  // Uncomment the line below if you want to use postgresql adapter
  // adapter: postgresAdapter({
  //   url: process.env.DATABASE_URL || "your-db-url",
  // }),
  plugins: [
    // Add plugins here
  ],
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },
  socialProviders: {
    // Add social providers if needed
  },
  user: {
    // Add custom fields for user background information
    additionalFields: {
      softwareExperience: {
        type: "string",
        required: false
      },
      hardwareExperience: {
        type: "string",
        required: false
      },
      roboticsBackground: {
        type: "string",
        required: false
      },
      aiMlExperience: {
        type: "string",
        required: false
      },
      preferences: {
        type: "string", // JSON string for user preferences
        required: false
      }
    }
  }
});